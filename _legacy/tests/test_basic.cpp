
#include <test_interface.h>

class AppExample : public mujoco::ITestApplication
{
    protected :

    void _initScenarioInternal() override;

    public :

    AppExample();
    ~AppExample();
};

AppExample::AppExample()
    : mujoco::ITestApplication()
{
    // nothing here
}

AppExample::~AppExample()
{
    // nothing here
}

void AppExample::_initScenarioInternal()
{
    // load the basic.xml mujoco scene
    m_mjcModelFile = std::string( TYSOC_PATH_WORKING_DIR ) + "basic.xml";
}

int main()
{

    auto _app = new AppExample();

    _app->init();
    _app->reset();

    while ( !_app->isTerminated() )
        _app->step();

    delete _app;

    return 0;
}