
#include <pytysoc_visualizer.h>

namespace py = pybind11;

namespace pytysoc
{


    PyVisualizer::PyVisualizer( tysoc::viz::TIVisualizer* visualizerPtr )
    {
        m_visualizerPtr = visualizerPtr;
    }

    PyVisualizer::~PyVisualizer()
    {
        m_visualizerPtr = NULL;
    }

    bool PyVisualizer::initialize()
    {
        if ( m_visualizerPtr )
            return m_visualizerPtr->initialize();

        std::cout << "ERROR> could not initialize non-existent "
                  << "wrapped visualizer object" << std::endl;
        return false;
    }

    void PyVisualizer::render()
    {
        if ( m_visualizerPtr )
        {
            m_visualizerPtr->update();
        }
        else
        {
            std::cout << "ERROR> could not request to render the scene on "
                      << "a non-existent wrapped visualizer object" << std::endl;
        }
    }

    bool PyVisualizer::isActive()
    {
        if ( m_visualizerPtr )
            return m_visualizerPtr->isActive();

        std::cout << "ERROR> could not request visualizer state on "
                  << "a non-existent wrapped visualizer object" << std::endl;
        return false;
    }

    bool PyVisualizer::isKeyDown( int keyCode )
    {
        if ( m_visualizerPtr )
            return m_visualizerPtr->isKeyDown( keyCode );

        std::cout << "ERROR> could not request isKeyDown on "
                  << "a non-existent wrapped visualizer object" << std::endl;
        return false;
    }

    bool PyVisualizer::checkSingleKeyPress( int keyCode )
    {
        if ( m_visualizerPtr )
            return m_visualizerPtr->checkSingleKeyPress( keyCode );

        std::cout << "ERROR> could not check single key press on "
                  << "a non-existent wrapped visualizer object" << std::endl;
        return false;
    }

    std::string PyVisualizer::type()
    {
        if ( m_visualizerPtr )
            return m_visualizerPtr->type();

        std::cout << "ERROR> could not request backend type on "
                  << "a non-existent wrapped visualizer object" << std::endl;
        return "undefined";
    }

}