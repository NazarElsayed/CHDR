#ifndef GRAPHICS_APPLICATION_HPP
#define GRAPHICS_APPLICATION_HPP

#include "ImGUI.hpp"

namespace Graphics {

    class Application {

    private:

        ImGUI m_GUI;

    public:
        int Run() {
            m_GUI.Initialise();

            bool exit = false;

            do {
                exit = m_GUI.Update();
            } while (!exit);

            m_GUI.CleanUp();

            return 0;
        }

    };
}

#endif //APPLICATION_HPP
