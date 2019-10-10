//
// Created by David Woller on 5.10.18.
//

#include "edge.h"

using namespace glns;

void Edge::draw(const Cairo::RefPtr<Cairo::Context> &cr) {
    cr->save();
    cr->set_line_width(0.5);
    cr->move_to(from.x, -from.y);
    cr->line_to(to.x, -to.y);
    cr->stroke();
    cr->restore();
}
