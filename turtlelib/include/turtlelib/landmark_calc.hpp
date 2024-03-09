/// \file landmark.hpp
/// \brief Landmark library to get x,y from polar coordinates for landmark and update the observation.


#include<iosfwd> // contains forward definitions for iostream objects
#include<armadillo> //Matrix library
#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"
#include"turtlelib/diff_drive.hpp"

namespace turtlelib
{
    class Landmarklib
    {
        private:
            std::vector<double> sensor_reading;
            std::vector<std::vector<Vector2D>> clusters;
            double angle_min;
            double angle_increment;

        public:
            /// \brief Default constructor.
            /// \details This constructor initializes the sensor reading vector.
            Landmarklib(std::vector<double> sensor_reading_, double angle_min_, double angle_increment);

            void cluster_laser_data();

            /// \brief Get the x and y coordinates from polar coordinates.
            /// \details This function takes in the polar coordinates and returns the x and y coordinates.
            /// \param r (double): The distance from the landmark.
            /// \param theta (double): The angle from the landmark.
            /// \return (arma::Col<double>): The x and y coordinates.
            Vector2D get_cartesian_coordinates(double r, double theta);

            /// \brief Get the clusters.
            /// \details This function returns the clusters.
            /// \return (arma::Mat<double>): The clusters.
             std::vector<std::vector<Vector2D>> get_clusters();

    };
}

