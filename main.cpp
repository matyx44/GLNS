#include <iostream>
#include <gtkmm/application.h>
#include <gtkmm/window.h>

#include "point.h"
#include "arc.h"
#include "parser.h"
#include "canvas.h"
#include "vertex.h"
#include "edge.h"
#include "set.h"
#include "tour.h"

#include <thread>


using namespace glns;

int main(int argc, char *argv[]) {
    // arcDrawingDemo();
    // randomTourDemo();


    std::vector<Arc> arcs;
    std::vector<Set> sets;
    Tour tour;

    auto app = Gtk::Application::create("gtkmm.glns");
    Gtk::Window win;
    win.set_title("GLNS");
    win.set_default_size(1200, 1000);

    Canvas canvas(arcs, sets, tour, argc, argv); // Planner->entryPoint is called in Canvas constructor

    win.add(canvas);
    canvas.show();


    return app->run(win);

}

