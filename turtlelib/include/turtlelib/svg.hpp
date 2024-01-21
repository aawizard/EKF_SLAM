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
        /// \param t the frame to draw the point in
        void draw_point( Point2D p, Transform2D t);

        /// \brief draw a point on the svg file with a specified color
        /// \param p the point to draw
        /// \param color the color of the point
        /// \param t the frame to draw the point in
        void draw_point( Point2D p, Transform2D t, std::string color);

        /// \brief draw a line on the svg file
        /// \param head the first point of the line
        /// \param tail the second point of the line
        /// \param t the frame to draw the line in
        void draw_line( Vector2D head,  Vector2D tail, Transform2D t);

        /// \brief draw a line on the svg file with a specified color
        /// \param head the first point of the line
        /// \param tail the second point of the line
        /// \param color the color of the line
        /// \param t the frame to draw the line in
        void draw_line( Vector2D head,  Vector2D tail, Transform2D t,  std::string color);

        /// \brief draw a frame the svg file 
        /// \param t the frame to draw
        /// \param name the name of the frame
        void draw_Transform2D( Transform2D t ,  std::string name);

        /// \brief export the svg file
        void export_svg();

        /// \brief export the svg file with a specified filename
        /// \param filename the name of the file to export
        void export_svg(std::string filename);

        private:
        std::string root=R"( <svg/>)";
    };
}