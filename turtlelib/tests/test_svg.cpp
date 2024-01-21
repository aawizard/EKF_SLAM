#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include"turtlelib/geometry2d.hpp"
#include"turtlelib/svg.hpp"
#include"turtlelib/se2d.hpp"
#include<sstream>

using namespace turtlelib;

TEST_CASE("Test SVG constructor", "[SVG]"){
    SVG svg;
    svg.draw_point(Point2D{4.0,3.0}, "purple");
    svg.draw_line(Point2D{1.5,1.5},Point2D{4.0,2.0} , "purple");
    svg.draw_Transform2D(Transform2D(Vector2D{1,1}, PI/2), "a");
    svg.export_svg();
    std::ifstream infile("my_drawing.svg");
    std::stringstream ss;
    ss << infile.rdbuf();
    std::string str = R"(
<svg width="8.500000in" height="11.000000in" viewBox="0 0 816.000000 1056.000000" xmlns="http://www.w3.org/2000/svg">

<defs>
  <marker style="overflow:visible" id="Arrow1Sstart" refX="0.0" refY="0.0" orient="auto">
       <path transform="scale(0.2) translate(6,0) " style="fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt"
         d="M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z "
       />
  </marker>
</defs>

<circle cx="792.000000" cy="240.000000" r="3" stroke="purple" fill="purple" stroke-width="1" />
          <line x1="792.000000" x2="552.000000" y1="336.000000" y2="384.000000" stroke="purple" stroke-width="5" marker-start="url(#Arrow1Sstart) " />
        <g>
          <line x1="504.000000" x2="504.000000" y1="336.000000" y2="432.000000" stroke="red" stroke-width="5" marker-start="url(#Arrow1Sstart) " />
          <line x1="408.000000" x2="504.000000" y1="432.000000" y2="432.000000" stroke="green" stroke-width="5" marker-start="url(#Arrow1Sstart) " />
        <text x="504.000000" y="432.000000">{a}</text>
        </g>
</svg>)";
    REQUIRE(ss.str() == str);
}