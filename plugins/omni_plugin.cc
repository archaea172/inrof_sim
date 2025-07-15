#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Inertial.hh>
#include <ignition/gazebo/components/ExternalForce.hh> // これを使用
#include <ignition/gazebo/components/ExternalTorque.hh> // これを使用

#include <ignition/transport/Node.hh>
#include <ignition/msgs/twist.pb.h>

#include <iostream>
#include <mutex>

namespace omni_plugini
{
    class OmniPlugin :  public ignition::gazebo::System,
                        public ignition::gazebo::ISystemConfigure,
                        public ignition::gazebo::ISystemPreUpdate
    {
        public OmniPlugin()
        : twistSubscribed(false), linearVelX(0.0), linearVelY(0.0), angularVelZ(0.0)
        {

        }
        public: void Configure(const ignition::gazebo::Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            ignition::gazebo::EntityComponentManager &_ecm,
                            ignition::gazebo::EventManager &_eventMgr) override
        {
        this->model = ignition::gazebo::Model(_entity);
        if (!this->model.Valid(_ecm))
        {
            ignerr << "OmniPlugin: Model entity not valid." << std::endl;
            return;
        }

        // SDFからパラメータを読み込む
        this->linkName = _sdf->Get<std::string>("link_name", "base_link").first;
        this->topicName = _sdf->Get<std::string>("topic_name", "/cmd_vel").first;
        this->linearForceMultiplier = _sdf->Get<double>("linear_force_multiplier", 10.0).first;
        this->angularForceMultiplier = _sdf->Get<double>("angular_force_multiplier", 5.0).first;
        this->hoverForceMultiplier = _sdf->Get<double>("hover_force_multiplier", 1.0).first; // 質量に応じて調整

        // リンクエンティティを取得
        this->linkEntity = this->model.LinkByName(_ecm, this->linkName);
        if (this->linkEntity == ignition::gazebo::kInvalidEntity)
        {
            ignerr << "OmniPlugin: Link [" << this->linkName << "] not found." << std::endl;
            return;
        }

        // Twistメッセージのサブスクライブ
        this->node.Subscribe(this->topicName, &OmniPlugin::OnTwist, this);
        ignmsg << "OmniPlugin: Subscribing to topic [" << this->topicName << "]" << std::endl;
        }
        public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                            ignition::gazebo::EntityComponentManager &_ecm) override
        {
        if (_info.paused)
            return;

        // リンクの質量を取得してホバー力を計算
        auto inertial = _ecm.Component<ignition::gazebo::components::Inertial>(this->linkEntity);
        double mass = 1.0; // デフォルト値
        if (inertial && inertial->Data().Mass())
        {
            mass = inertial->Data().Mass().value();
        }

        // 重力加速度を取得 (デフォルトは9.8)
        double gravity = 9.81; // Gazeboのデフォルト重力
        auto worldEntity = _ecm.Component<ignition::gazebo::components::World>(this->model.Entity());
        if (worldEntity) {
            auto gravityComp = _ecm.Component<ignition::gazebo::components::Gravity>(worldEntity->Data());
            if (gravityComp) {
                gravity = gravityComp->Data().Z(); // Z軸方向の重力加速度
                if (gravity < 0) gravity = -gravity; // 絶対値
            }
        }

        // ホバー力を適用 (重力に抗する力)
        // ワールド座標系での上向きの力
        ignition::math::Vector3d hoverForce(0, 0, mass * gravity * this->hoverForceMultiplier);
        _ecm.CreateComponent(this->linkEntity, ignition::gazebo::components::ExternalWorldForce(hoverForce));


        // Twistメッセージから力を計算
        std::lock_guard<std::mutex> lock(this->mtx);
        ignition::math::Vector3d linearForce(
            this->linearVelX * this->linearForceMultiplier,
            this->linearVelY * this->linearForceMultiplier,
            0.0 // Z方向はホバー力で制御
        );
        ignition::math::Vector3d angularTorque(
            0.0,
            0.0,
            this->angularVelZ * this->angularForceMultiplier
        );

        // 力をリンクのローカル座標系で適用
        // NOTE: ApplyForce/Torqueはワールド座標系だが、Twistは通常ローカル座標系で解釈される
        // ここでは簡易的に、TwistのX/YをリンクのX/Y方向の力、ZをリンクのZ軸周りのトルクとして適用
        // より正確には、リンクの現在の姿勢を取得し、Twistをワールド座標系に変換して力を適用するか、
        // リンクに直接力を加えるGazebo APIを使用する必要がある。
        // 現在のGazebo APIでは、ExternalWorldForce/Torqueが最も直接的な方法。
        // ローカル座標系での力適用は、Link::AddForce(relative_to_link=true) のような機能が必要だが、
        // Systemプラグインでは直接利用できないため、姿勢変換が必要になる。
        // 今回は簡易的に、ワールド座標系での力として適用する。
        // ホバークラフトの特性上、Z軸周りの回転はローカルZ軸トルク、X/Y移動はローカルX/Y力で妥当。

        // リンクの現在の姿勢を取得
        auto linkPoseComp = _ecm.Component<ignition::gazebo::components::Pose>(this->linkEntity);
        if (linkPoseComp)
        {
            ignition::math::Pose3d linkPose = linkPoseComp->Data();
            ignition::math::Quaterniond linkRot = linkPose.Rot();

            // ローカル座標系の力をワールド座標系に変換
            ignition::math::Vector3d worldLinearForce = linkRot.RotateVector(linearForce);
            ignition::math::Vector3d worldAngularTorque = linkRot.RotateVector(angularTorque); // トルクも回転させる

            _ecm.CreateComponent(this->linkEntity, ignition::gazebo::components::ExternalWorldForce(worldLinearForce));
            _ecm.CreateComponent(this->linkEntity, ignition::gazebo::components::ExternalWorldTorque(worldAngularTorque));
        }
        }

        private: void OnTwist(const ignition::msgs::Twist &_msg)
        {
        std::lock_guard<std::mutex> lock(this->mtx);
        this->linearVelX = _msg.linear().x();
        this->linearVelY = _msg.linear().y();
        this->angularVelZ = _msg.angular().z();
        this->twistSubscribed = true;
        }

        private: ignition::gazebo::Model model;
        private: ignition::gazebo::Entity linkEntity;
        private: std::string linkName;
        private: std::string topicName;
        private: double linearForceMultiplier;
        private: double angularForceMultiplier;
        private: double hoverForceMultiplier;

        private: ignition::transport::Node node;
        private: std::mutex mtx;
        private: bool twistSubscribed;
        private: double linearVelX;
        private: double linearVelY;
        private: double angularVelZ;
    };

    // プラグインを登録
    IGNITION_ADD_PLUGIN(OmniPlugin,
                        ignition::gazebo::System,
                        OmniPlugin::ISystemConfigure,
                        OmniPlugin::ISystemPreUpdate)

    IGNITION_ADD_PLUGIN_ALIAS(OmniPlugin, "omni_plugin")
}