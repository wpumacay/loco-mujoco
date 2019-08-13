
#include <test_interface.h>

class AppExample : public mujoco::ITestApplication
{
    private :

    int                             m_currentBodyIndx;
    std::string                     m_currentBodyName;
    std::vector< mujoco::SimBody* > m_bodies;
    std::vector< float >            m_bodiesBaseSizeX;
    std::vector< float >            m_bodiesBaseSizeY;
    std::vector< float >            m_bodiesBaseSizeZ;

    protected :

    void _initScenarioInternal() override;
    void _onApplicationStart() override;
    void _renderUiInternal() override;

    public :

    AppExample();
    ~AppExample();
};

AppExample::AppExample()
    : mujoco::ITestApplication()
{
    m_currentBodyIndx = -1;
    m_currentBodyName = "";
}

AppExample::~AppExample()
{
    // nothing here
}

void AppExample::_initScenarioInternal()
{
    // load the basic.xml mujoco scene
    m_mjcModelFile = std::string( TYSOC_PATH_WORKING_DIR ) + "basic_terrain.xml";
}

void AppExample::_onApplicationStart()
{
    float _sizex, _sizey, _sizez;

    auto _simBody1 = getBody( "bbox1" );
    m_bodies.push_back( _simBody1 );
    // grab the current size of this body
    m_bodiesBaseSizeX.push_back( m_mjcModelPtr->geom_size[ 3 * _simBody1->geomsIds().front() + 0 ] );
    m_bodiesBaseSizeY.push_back( m_mjcModelPtr->geom_size[ 3 * _simBody1->geomsIds().front() + 1 ] );
    m_bodiesBaseSizeZ.push_back( m_mjcModelPtr->geom_size[ 3 * _simBody1->geomsIds().front() + 2 ] );

    auto _simBody2 = getBody( "bbox2" );
    m_bodies.push_back( _simBody2 );
    // grab the current size of this body
    m_bodiesBaseSizeX.push_back( m_mjcModelPtr->geom_size[ 3 * _simBody2->geomsIds().front() + 0 ] );
    m_bodiesBaseSizeY.push_back( m_mjcModelPtr->geom_size[ 3 * _simBody2->geomsIds().front() + 1 ] );
    m_bodiesBaseSizeZ.push_back( m_mjcModelPtr->geom_size[ 3 * _simBody2->geomsIds().front() + 2 ] );
}

void AppExample::_renderUiInternal()
{
    ImGui::Begin( "Playground-primitives" );

    if ( ImGui::BeginCombo( "Bodies", m_currentBodyName.c_str() ) )
    {
        for ( int i = 0; i < m_bodies.size(); i++ )
        {
            if ( !m_bodies[i] )
                continue;

            bool _isSelected = ( i == m_currentBodyIndx );
            if ( ImGui::Selectable( m_bodies[i]->name().c_str(), _isSelected ) )
            {
                m_currentBodyIndx = i;
                m_currentBodyName = m_bodies[i]->name();
            }

            if ( _isSelected )
                ImGui::SetItemDefaultFocus();
        }

        ImGui::EndCombo();
    }

    if ( m_currentBodyIndx != -1 )
    {
        ImGui::Spacing();
        {
            auto _simBody = m_bodies[m_currentBodyIndx];
            auto _bodyId = _simBody->id();
            auto _bodyName = _simBody->name();

            // assumption: these bodies have only one geom
            auto _bodyGeomId = _simBody->geomsIds().front();
            auto _bodyGeomGraphics = _simBody->geomsGraphics().front();

            // grab the current size of this body
            float _sizex = m_mjcModelPtr->geom_size[ 3 * _bodyGeomId + 0 ];
            float _sizey = m_mjcModelPtr->geom_size[ 3 * _bodyGeomId + 1 ];
            float _sizez = m_mjcModelPtr->geom_size[ 3 * _bodyGeomId + 2 ];

            // create a slider per dimension
            ImGui::SliderFloat( ( _bodyName + "-size-x" ).c_str(), &_sizex, 0.01, 1.0 );
            ImGui::SliderFloat( ( _bodyName + "-size-y" ).c_str(), &_sizey, 0.01, 1.0 );
            ImGui::SliderFloat( ( _bodyName + "-size-z" ).c_str(), &_sizez, 0.01, 1.0 );

            // for test for cubes, we have to change the rbound, so size is 3x1
            float _rBound = tysoc::TVec3::length( { _sizex, _sizey, _sizez } );

            // set new properties given by the user-ui
            m_mjcModelPtr->geom_size[ 3 * _bodyGeomId + 0 ] = _sizex;
            m_mjcModelPtr->geom_size[ 3 * _bodyGeomId + 1 ] = _sizey;
            m_mjcModelPtr->geom_size[ 3 * _bodyGeomId + 2 ] = _sizez;

            m_mjcModelPtr->geom_rbound[_bodyGeomId] = _rBound;

            // Update the properties of the graphics objects
            float _scalex = _sizex / m_bodiesBaseSizeX[m_currentBodyIndx];
            float _scaley = _sizey / m_bodiesBaseSizeY[m_currentBodyIndx];
            float _scalez = _sizez / m_bodiesBaseSizeZ[m_currentBodyIndx];

            reinterpret_cast< engine::LMesh* >( _bodyGeomGraphics )->scale = { _scalex, _scaley, _scalez };
        }
    }

    ImGui::End();
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