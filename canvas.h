//
// Created by David Woller on 20.9.18.
//

#ifndef GLNS_CANVAS_H
#define GLNS_CANVAS_H


#include <gtkmm/drawingarea.h>
#include <giomm.h>
#include <thread>
#include "arc.h"
#include "set.h"
#include "tour.h"
#include "planner.h"

namespace glns {

class Canvas : public Gtk::DrawingArea
{
public:
    explicit Canvas(std::vector<Arc> arcs, std::vector<Set> sets, Tour tour, int argc, char *argv[]);
    ~Canvas() override;
    void setArcs(std::vector<Arc> arcs);
    void setSets(std::vector<Set> sets);
    void setTour(Tour tour);
    void setBestTour(Tour tour);
    void notify();
    void onNotification();
    // void thread(Canvas* caller);


private:
    std::vector<Arc> arcs;
    std::vector<Set> sets;
    Tour tour;
    Tour bestTour;
    double scale;
    Point grab;
    Point shift;

    Planner planner;
    std::thread* plannerThread;
    Glib::Dispatcher Dispatcher;
    // std::thread* m_WorkerThread;


protected:
    //Override default signal handler:
    bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr) override;
    bool on_scroll_event(GdkEventScroll *event) override;
    bool on_button_press_event(GdkEventButton* event) override;
    bool on_button_release_event(GdkEventButton* event) override;
    bool on_motion_notify_event(GdkEventMotion* event) override;


};

}

#endif //GLNS_CANVAS_H
