/// \file landmark_calc.hpp
/// \brief Landmark library to get x,y from polar coordinates for landmark and update the observation.


#include<iosfwd> // contains forward definitions for iostream objects
#include<vector>
#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"
#include"turtlelib/diff_drive.hpp"

namespace turtlelib
{
    /// \brief Landmark library to get x,y of landmarks from sensor data.
    class Landmarklib
    {
        private:
            std::vector<double> sensor_reading;
            std::vector<std::vector<Vector2D>> clusters;
            double angle_min;
            double angle_increment;
            std::vector<std::vector<double>> landmark_centroids;

        public:
            /// \brief Default constructor.
            /// \details This constructor initializes the sensor reading vector.
            explicit Landmarklib(std::vector<double> sensor_reading_, double angle_min_, double angle_increment);

            /// \brief Constructor.
            /// \details This constructor initializes the clusters.
            explicit Landmarklib(std::vector<std::vector<Vector2D>> clusters_);
            
            /// \brief Cluster the laser data.
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

            /// \brief Get the landmarks centroids.
            /// \details This function returns the centroids of the clusters.
            /// \return (std::vector<std::vector<double>>): The centroids of the clusters.
            std::vector<std::vector<double>>  get_landmark_centroids();

            /// \brief Fit a circle to the cluster.
            /// \details This function fits a circle to the cluster.
            /// \param cluster (std::vector<Vector2D>): The cluster.
            /// \return (Vector2D): The centroid of the cluster.
            std::vector<double> circle_fitting(std::vector<Vector2D> cluster);



    };
}

