#include <iostream>
#include <inria_maths/ControlProportionalBarrier.hpp>
#include <inria_plot/Plot.hpp>

int main()
{
    inria::ControlProportionalBarrier controller;
    controller.setParameters(10.0, 4.0, -0.5, 1.0, 0.2);

    inria::Plot plot;
    for (double q=-1.5;q<1.5;q+=0.001) {
        double effort = controller.computeControlEffort(0.5, q);
        double gain = controller.computeControlGain(0.5, q);
        plot.add(
            "effort", effort,
            "gain", gain,
            "position", q
        );
    }
    plot.plot("position", "all").show();

    return 0;
}

