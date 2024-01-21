#include <turtlelib/svg.hpp>
#include<iosfwd> 
#include <fstream>
#include<iostream>
#include<cmath>
#include"turtlelib/se2d.hpp"
using namespace std;


namespace turtlelib{
    SVG::SVG(){
        std::cout<<"SVG constructor"<<std::endl;
        root=R"(
<svg width="8.500000in" height="11.000000in" viewBox="0 0 816.000000 1056.000000" xmlns="http://www.w3.org/2000/svg">

<defs>
  <marker style="overflow:visible" id="Arrow1Sstart" refX="0.0" refY="0.0" orient="auto">
       <path transform="scale(0.2) translate(6,0) " style="fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt"
         d="M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z "
       />
  </marker>
</defs>

)";



// </svg>
        std::ofstream outfile("my_drawing.svg");
        outfile << std::string(root);
    }


    void SVG::draw_point(const Point2D p){
        std::cout<<"draw_point"<<std::endl;
        root+=R"(<circle cx=")";
        // turtlelib::Point2D pixel_pt;
        // pixel_pt.x = pt.x *96.0 + 8.5/2.0*96.0;
        // pixel_pt.y = -pt.y *96.0 + 11.0/2.0*96.0;
        root+=to_string(p.x*96.0 + 8.5/2.0*96.0);
        // 504.2
        root+=R"(" cy=")";
        // 403.5
        root+=to_string(-p.y*96.0 + 11.0/2.0*96.0);
        root+=R"(" r="3" stroke="purple" fill="purple" stroke-width="1" />)";
    }
    void SVG::draw_point(const Point2D p, const std::string color){
        std::cout<<"draw_point"<<std::endl;
        root+=R"(<circle cx=")";
        root+=to_string(p.x*96.0 + 8.5/2.0*96.0);
        // 504.2
        root+=R"(" cy=")";
        // 403.5
        root+=to_string(-p.y*96.0 + 11.0/2.0*96.0);
        root+=R"(" r="3" stroke=")";
        root+=color;
        root+=R"(" fill="purple" stroke-width="1" />)";
    }



    void SVG::draw_line( Point2D p1,  Point2D p2){

        std::cout<<"draw_line"<<std::endl;
        root+=R"(
          <line x1=")";
        root+=to_string(p1.x*96.0 + 8.5/2.0*96.0);
        root+=R"(" x2=")";
        root+=to_string(p2.x*96.0 + 8.5/2.0*96.0);
        root+=R"(" y1=")";
        root+=to_string(-p1.y*96.0 + 11.0/2.0*96.0);
        root+=R"(" y2=")";
        root+=to_string(-p2.y*96.0 + 11.0/2.0*96.0);
        root+=R"(" stroke="purple" stroke-width="5" marker-start="url(#Arrow1Sstart) " />)";
    }
     void SVG::draw_line( Point2D p1,  Point2D p2,   std::string color){

        std::cout<<"draw_line"<<std::endl;
        root+=R"(
          <line x1=")";
        root+=to_string(p2.x*96.0 + 8.5/2.0*96.0);
        root+=R"(" x2=")";
        root+=to_string(p1.x*96.0 + 8.5/2.0*96.0);
        root+=R"(" y1=")";
        root+=to_string(-p2.y*96.0 + 11.0/2.0*96.0);
        root+=R"(" y2=")";
        root+=to_string(-p1.y*96.0 + 11.0/2.0*96.0);
        root+=R"(" stroke=")";
        root+=color;
        root+=R"(" stroke-width="5" marker-start="url(#Arrow1Sstart) " />)";
      }

    void SVG::draw_Transform2D( Transform2D t ,  std::string name){
      Point2D x{1.0,0};
      Point2D y{0,1.0};
      Point2D x_ = t(x);
      Point2D y_ = t(y);

      
      // <g>
//   <line x1="504.000000" x2="408.000000" y1="528.000000" y2="528.000000" stroke="red" stroke-width="5" marker-start="url(#Arrow1Sstart) " />
//   <line x1="408.000000" x2="408.000000" y1="432.000000" y2="528.000000" stroke="green" stroke-width="5" marker-start="url(#Arrow1Sstart) " />
//   <text x="408.000000" y="528.250000">{a}</text>
// </g>

      root+=R"(
        <g>)";
      Point2D origin{t.translation().x, t.translation().y};

      draw_line( origin,x_, "red");
      draw_line( origin,y_, "green");
      root+=R"(
        <text x=")";
      root+=to_string(t.translation().x*96.0 + 8.5/2.0*96.0);
      root+=R"(" y=")";
      root+=to_string(-t.translation().y*96.0 + 11.0/2.0*96.0);
      root+=R"(">{)";
      root+=name;
      root+=R"(}</text>)";
      root+=R"(
        </g>)";
    }

    void SVG::export_svg(){
        std::cout<<"export_svg"<<std::endl;
        root+=R"(
</svg>)";
        std::ofstream outfile("my_drawing.svg");
        outfile << std::string(root);
    }
}


// int main() {
//     // SVG::SVG root;
//     // // Basic CSS support
//     // root.style("circle").set_attr("fill", "#000000")
//     //     .set_attr("stroke", "#000000");
//     // root.style("rect#my_rectangle").set_attr("fill", "red");
//     // // Method 1 of adding elements - add_child<>()
//     // auto shapes = root.add_child<SVG::Group>();
//     // auto rect = shapes->add_child<SVG::Rect>("my_rectangle");
//     // // Method 2 of adding elements - operator<<
//     // *shapes << SVG::Circle(-100, -100, 100) << SVG::Circle(100, 100, 100);
//     // // Reference elements by id, tag, class name, etc...
//     // root.get_element_by_id("my_rectangle")
//     //     ->set_attr("x", 20).set_attr("y", 20)
//     //     .set_attr("width", 40).set_attr("height", 40);
//     // std::cout << "There are " << root.get_children<SVG::Circle>().size() <<
//     //     " circles." << std::endl;
//     // // Automatically scale width and height to fit elements
//     // root.autoscale();
//     // // Output our drawing
//     std::ofstream outfile("my_drawing.svg");
//     outfile << std::string(root);
// }