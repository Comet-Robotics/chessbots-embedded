#pragma once

class Coordinate2D {
    public:
        double x;
        double y;
    
        Coordinate2D();

        // Unit vector from angle
        Coordinate2D(double angle);
        Coordinate2D(double x, double y);

        // Transform (same as addition)
        Coordinate2D transform(Coordinate2D offset) const;

        // Transform with polar coordinate
        Coordinate2D transform(double theta, double distance) const;

        // Rotate around origin
        Coordinate2D rotate(double theta) const;
        // Scale (same as multiplication)
        Coordinate2D scale(double theta) const;

        double distance_to(const Coordinate2D destination) const;
        double angle_to(const Coordinate2D destination) const;
        Coordinate2D vector_to(const Coordinate2D destination) const;

        double dot_product(const Coordinate2D other) const;
        bool is_behind(double radians, Coordinate2D other) const;
};