#include <math.h>
#include "utils/geometry.h"

Coordinate2D::Coordinate2D() {
    x = 0.0;
    y = 0.0;
}

Coordinate2D::Coordinate2D(double _x, double _y) {
    x = _x;
    y = _y;
}

Coordinate2D::Coordinate2D(double angle) {
    x = cos(angle);
    y = sin(angle);
}

// Coordinate2D operator+(const Coordinate2D& a, const Coordinate2D& b) { return a.transform(b); }
Coordinate2D Coordinate2D::transform(const Coordinate2D offset) const {
    return Coordinate2D(x + offset.x, y + offset.y);
}

Coordinate2D Coordinate2D::transform(double theta, double distance) const {
    return Coordinate2D(x + distance * cos(theta), y + distance * sin(theta));
}

Coordinate2D Coordinate2D::rotate(double theta) const {
    double newX = x * cos(theta) - y * sin(theta);
    double newY = x * sin(theta) + y * cos(theta);
    return Coordinate2D(newX, newY);
}

// Coordinate2D operator*(const Coordinate2D& a, double factor) { return a.scale(factor); }
Coordinate2D Coordinate2D::scale(double factor) const {
    return Coordinate2D(x * factor, y * factor);
}

double Coordinate2D::distance_to(const Coordinate2D destination) const {
    double dx = destination.x - x;
    double dy = destination.y - y;
    return pow(
        pow(dy, 2) + pow(dx, 2),
        .5
    );
}

double Coordinate2D::angle_to(const Coordinate2D destination) const {
    double dx = destination.x - x;
    double dy = destination.y - y;
    return atan2(dy, dx);
}

// Same as destination - self
Coordinate2D Coordinate2D::vector_to(const Coordinate2D destination) const {
    Coordinate2D c(destination.x - x, destination.y - y);

    return c;
}

double Coordinate2D::dot_product(const Coordinate2D other) const {
    return (x * other.x) + (y * other.y);
}

bool Coordinate2D::is_behind(double radians, Coordinate2D other) const {
    return vector_to(other).dot_product(Coordinate2D(radians)) < 0;
}