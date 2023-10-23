#include <iostream>
#include <inria_plot/Plot.hpp>

int main()
{
    inria::Plot plot;
    for (int i=0;i<100;i++) {
        plot.add(
            "aa:toto", i,
            "aa:titi", -i,
            "bb:tata", 1.0,
            "bb:tete", -0.5*i
        );
    }
    plot.add("cc:tutu", 0.0);
    plot.add("cc:tutu", 0.0);
    plot
        .rangeX(0.0, 120.0).rangeY(-20.0, 20.0)
        .plot("index", "all")
        .plot("index", "aa:*", inria::Plot::Lines)
        .plot("index", "aa:titi", inria::Plot::Points)
        .plot("index", "bb:tata", inria::Plot::Points, "aa:toto")
        .show();
    plot
        .rangeX(0.0, 120.0).rangeY(-20.0, 20.0)
        .plot("index", "all")
        .plot("index", "aa:titi", inria::Plot::Points)
        .plot("index", "bb:tata", inria::Plot::Points, "aa:toto")
        .show("/tmp/testPlot.plot");
    plot
        .rangeUniform()
        .plot("index", "all")
        .show();

    plot.add(
        "t", 1.0,
        "val", 1.0
    );
    plot.add(
        "t", 2.0,
        "val", 3.0
    );
    plot.add(
        "t", 3.0,
        "val", 5.0
    );
    plot
        .rangeX(0.0, 120.0).rangeY(-20.0, 20.0)
        .multiplot(1, 2)
        .plot("index", "bb:tata", inria::Plot::Points, "aa:toto")
        .nextPlot()
        .plot("index", "aa:titi", inria::Plot::Points)
        .nextPlot()
        .plot("t", "val")
        .plot("index", "aa:toto", inria::Plot::Points)
        .show();
    plot.writeData();

    plot
        .plot("aa:toto", "aa:titi", "bb:tete", inria::Plot::Points)
        .plot("aa:titi", "aa:toto", "bb:tete", inria::Plot::LinesPoints, "bb:tete")
        .show();

    return 0;
}

