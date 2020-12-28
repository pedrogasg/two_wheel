#ifndef TWO_WHEEL_SIMULATION__TWO_WHEEL_PLUGIN
#define TWO_WHEEL_SIMULATION__TWO_WHEEL_PLUGIN

#include <gazebo/common/Plugin.hh>

namespace gazebo_plugins
{
    class TwoWheelPluginPrivate;


    class TwoWheelPlugin: public gazebo::ModelPlugin
    {
    
    public:
    
        TwoWheelPlugin(/* args */);
    
        ~TwoWheelPlugin();

    protected:
    
        void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

        //void Reset() override;

    private:

        std::unique_ptr<TwoWheelPluginPrivate> impl_;  

    };


    
} // namespace gazebo_plugins


#endif //TWO_WHEEL_SIMULATION__TWO_WHEEL_PLUGIN