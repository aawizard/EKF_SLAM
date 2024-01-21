#include<iosfwd> // contains forward definitions for iostream objects
#include <fstream>

#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"
namespace turtlelib
{
    /// \brief Class to create an SVG file
    class SVG{
        public:
        /// \brief constructor to initialize the svg file
        SVG();
        /// \brief draw a point on the svg file
        /// \param p the point to draw
        void draw_point(const Point2D p);

        /// \brief draw a point on the svg file with a specified color
        /// \param p the point to draw
        /// \param color the color of the point
        void draw_point(const Point2D p, const std::string color);

        /// \brief draw a line on the svg file
        /// \param p1 the first point of the line
        /// \param p2 the second point of the line
        void draw_line( Point2D p1,  Point2D p2);

        /// \brief draw a line on the svg file with a specified color
        /// \param p1 the first point of the line
        /// \param p2 the second point of the line
        /// \param color the color of the line
        void draw_line( Point2D p1,  Point2D p2,  std::string color);

        /// \brief draw a frame the svg file 
        /// \param t the frame to draw
        /// \param name the name of the frame
        void draw_Transform2D( Transform2D t ,  std::string name);

        /// \brief export the svg file
        void export_svg();

        private:
        std::string root=R"( <svg/>)";
    };
}