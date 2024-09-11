#include "core/Application.hpp"
#include "core/ImGUI.hpp"

int main() {

    int exitCode = 0;

    Graphics::Application app;

    exitCode = app.Run();

    return exitCode;
}