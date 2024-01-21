#include <turtlelib/svg.hpp>
#include<iosfwd> 
#include <fstream>
#include<iostream>
#include<cmath>
#include"turtlelib/se2d.hpp"
using namespace std;


namespace turtlelib{
    SVG::SVG(){
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
        std::ofstream outfile("my_drawing.svg");
        outfile << std::string(root);
    }


    void SVG::draw_point( Point2D p, Transform2D t){

        Point2D pt = t(p);
        root+=R"(<circle cx=")";
        root+=to_string(pt.x*96.0 + 8.5/2.0*96.0);
        // 504.2
        root+=R"(" cy=")";
        // 403.5
        root+=to_string(-pt.y*96.0 + 11.0/2.0*96.0);
        root+=R"(" r="3" stroke="purple" fill="purple" stroke-width="1" />)";
    }
    void SVG::draw_point( Point2D p, Transform2D t, const std::string color){
        Point2D pt = t(p);
        root+=R"(<circle cx=")";
        root+=to_string(pt.x*96.0 + 8.5/2.0*96.0);
        // 504.2
        root+=R"(" cy=")";
        // 403.5
        root+=to_string(-pt.y*96.0 + 11.0/2.0*96.0);
        root+=R"(" r="3" stroke=")";
        root+=color;
        root+=R"(" fill="purple" stroke-width="1" />)";
    }



    void SVG::draw_line( Vector2D head,  Vector2D tail, Transform2D t){
      Point2D head1{head.x, head.y};
      Point2D tail1{tail.x, tail.y}; 
      Point2D head_ = t(head1);
      Point2D tail_ = t(tail1);

        root+=R"(
          <line x1=")";
        root+=to_string(head_.x*96.0 + 8.5/2.0*96.0);
        root+=R"(" x2=")";
        root+=to_string(tail_.x*96.0 + 8.5/2.0*96.0);
        root+=R"(" y1=")";
        root+=to_string(-head_.y*96.0 + 11.0/2.0*96.0);
        root+=R"(" y2=")";
        root+=to_string(-tail_.y*96.0 + 11.0/2.0*96.0);
        root+=R"(" stroke="purple" stroke-width="5" marker-start="url(#Arrow1Sstart) " />)";
    }
     void SVG::draw_line( Vector2D head,  Vector2D tail, Transform2D t,  std::string color){
      Point2D head1{head.x, head.y};
      Point2D tail1{tail.x, tail.y}; 
      Point2D head_ = t(head1);
      Point2D tail_ = t(tail1);
        root+=R"(
          <line x1=")";
        root+=to_string(head_.x*96.0 + 8.5/2.0*96.0);
        root+=R"(" x2=")";
        root+=to_string(tail_.x*96.0 + 8.5/2.0*96.0);
        root+=R"(" y1=")";
        root+=to_string(-head_.y*96.0 + 11.0/2.0*96.0);
        root+=R"(" y2=")";
        root+=to_string(-tail_.y*96.0 + 11.0/2.0*96.0);
        root+=R"(" stroke=")";
        root+=color;
        root+=R"(" stroke-width="5" marker-start="url(#Arrow1Sstart) " />)";
      }

    void SVG::draw_Transform2D( Transform2D t ,  std::string name){

      root+=R"(
        <g>)";
      

      draw_line( Vector2D{1,0},Vector2D{0,0}, t ,"red");
      draw_line( Vector2D{0,1},Vector2D{0,0}, t , "green");
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
        root+=R"(
</svg>)";
        std::ofstream outfile("my_drawing.svg");
        outfile << std::string(root);
    }

    void SVG::export_svg(std::string filename){
        root+=R"(
</svg>)";
        std::ofstream outfile(filename);
        outfile << std::string(root);
    }
}
