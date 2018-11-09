
#include "mjcf.h"


#ifndef MUJOCO_RESOURCES_PATH
    #define MUJOCO_RESOURCES_PATH "../../res/"
#endif

void traverse( mjcf::IElement* element, int depth = 0 );

int main()
{
    std::string _modelPath( MUJOCO_RESOURCES_PATH );
    _modelPath += "mjxml/basic.xml";

    auto _root = mjcf::loadModel( _modelPath );

    traverse( _root );

    mjcf::saveModel( _root, "test_basic.xml" );

    auto _root2 = new mjcf::IElement();
    _root2->etype = "mujoco";

    {
        auto _worldbody1 = new mjcf::IElement();
        _worldbody1->etype = "worldbody";

        {
            auto _light = new mjcf::ELight();
            _light->etype = "light";
            _light->diffuse = { 0.7, 0.7, 0.7 };
            _light->pos = { 0.0, 0.0, 4 };
            _light->dir = { 0.0, 0.0, -1.0 };

            auto _plane = new mjcf::EGeom();
            _plane->etype = "geom";
            _plane->type = "plane";
            _plane->size = { 3, { 2.0, 2.0, 0.2 } };
            _plane->rgba = { 0.9, 0.1, 0.2, 1.0 };
            
            auto _body1 = new mjcf::EBody();
            _body1->etype = "body";
            _body1->pos = { 0.0, 0.0, 2.0 };

            auto _joint1 = new mjcf::EJoint();
            _joint1->etype = "joint";
            _joint1->type = "free";

            auto _body1geom = new mjcf::EGeom();
            _body1geom->etype = "geom";
            _body1geom->type = "box";
            _body1geom->size = { 3, { 0.15, 0.15, 0.15 } };
            _body1geom->rgba = { 0.0, 0.9, 0.0, 1.0 };

            _worldbody1->children.push_back( _light );
            _worldbody1->children.push_back( _plane );
            _worldbody1->children.push_back( _body1 );

            _body1->children.push_back( _joint1 );
            _body1->children.push_back( _body1geom );
        }

        _root2->children.push_back( _worldbody1 );
    }

    {
        auto _worldbody2 = new mjcf::IElement();
        _worldbody2->etype = "worldbody";

        {
            auto _body2 = new mjcf::EBody();
            _body2->etype = "body";
            _body2->pos = { 0.5, 0.5, 2.0 };

            auto _joint2 = new mjcf::EJoint();
            _joint2->etype = "joint";
            _joint2->type = "free";

            auto _body2geom = new mjcf::EGeom();
            _body2geom->etype = "geom";
            _body2geom->type = "sphere";
            _body2geom->size = { 1, { 0.25 } };
            _body2geom->rgba = { 0.2, 0.9, 0.2, 1.0 };

            _worldbody2->children.push_back( _body2 );
            
            _body2->children.push_back( _joint2 );
            _body2->children.push_back( _body2geom );
        }

        _root2->children.push_back( _worldbody2 );
    }

    mjcf::saveModel( _root2, "test_basic_programmatical.xml" );

    return 0;
}

void traverse( mjcf::IElement* element, int depth )
{
    for ( size_t i = 0; i < depth; i++ )
    {
        std::cout << "\t";
    }
    // std::cout << "el: " << element->etype << std::endl;
    element->print();

    for ( size_t i = 0; i < element->children.size(); i++ )
    {
        traverse( element->children[i], depth + 1 );
    }
}