/*!******************************************************************************
 * \authors   David Eugenio <david.eugenioq@alumnos.upm.es>
 * \copyright Copyright (c) 2022 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#if !defined(GEOMETRY_POINT_HEADER_IS_INCLUDED)
#define GEOMETRY_POINT_HEADER_IS_INCLUDED

#include <array>
#include <cmath>
#include <ostream>

#include "utility.hpp"

#include "geometry/distance.hpp"

namespace gmt {

template <std::size_t Dimension>
class point {
public:
    static_assert(Dimension > 0u);

    template <typename... Distance>
    point(Distance... component) : m_components{component...} { }

    std::size_t dimension() const { return Dimension; }

    distance &operator[](std::size_t index) { return m_components[index]; }

    const distance &operator[](std::size_t index) const { return m_components[index]; }

    point &operator+=(const point &point);

    point &operator-=(const point &point);

    point &operator*=(distance distance);

    point &operator/=(distance distance);

private:
    std::array<distance, Dimension> m_components;
};

}



namespace std {

template <size_t Dimension>
struct tuple_size<gmt::point<Dimension>> : integral_constant<size_t, Dimension> { };



template <size_t Index, size_t Dimension>
struct tuple_element<Index, gmt::point<Dimension>> {
    using type = gmt::distance;
};



template <size_t Index, size_t Dimension>
gmt::distance &get(gmt::point<Dimension> &point) {
    return point[Index];
}

template <size_t Index, size_t Dimension>
const gmt::distance &get(const gmt::point<Dimension> &point) {
    return point[Index];
}

}



namespace gmt {

template <std::size_t Index, std::size_t Dimension>
distance &get(point<Dimension> &point) {
    return point[Index];
}

template <std::size_t Index, std::size_t Dimension>
const distance &get(const point<Dimension> &point) {
    return point[Index];
}



template <std::size_t Dimension>
point<Dimension> operator+(const point<Dimension> &point_0, const point<Dimension> &point_1) {
    auto auxiliar = point_0;

    auxiliar += point_1;

    return auxiliar;
}

template <std::size_t Dimension>
point<Dimension> operator-(const point<Dimension> &point_0, const point<Dimension> &point_1) {
    auto auxiliar = point_0;

    auxiliar -= point_1;

    return auxiliar;
}



template <std::size_t Dimension>
point<Dimension> operator*(const point<Dimension> &point, distance distance) {
    auto auxiliar = point;

    auxiliar *= distance;

    return auxiliar;
}

template <std::size_t Dimension>
point<Dimension> operator*(distance distance, const point<Dimension> &point) {
    return point * distance;
}

template <std::size_t Dimension>
point<Dimension> operator/(const point<Dimension> &point, distance distance) {
    auto auxiliar = point;

    auxiliar /= distance;

    return auxiliar;
}



template <std::size_t Dimension>
bool operator==(const point<Dimension> &point_0, const point<Dimension> &point_1) {
    for(std::size_t i = 0u; i < Dimension; ++i) {
        if(point_0[i] != point_1[i]) {
            return false;
        }
    }

    return true;
}

template <std::size_t Dimension>
bool operator!=(const point<Dimension> &point_0, const point<Dimension> &point_1) {
    return !(point_0 == point_1);
}



template <std::size_t Dimension>
std::ostream &operator<<(std::ostream &stream, const point<Dimension> &point) {
    stream << '(';

    for(std::size_t i = 0u; i < point.dimension(); ++i) {
        stream << point[i];

        if(i != point.dimension() - 1u) {
            stream << ',' << ' ';
        }
    }

    return stream << ')';
}



template <std::size_t Dimension>
point<Dimension> &point<Dimension>::operator+=(const point<Dimension> &point) {
    for(std::size_t i = 0u; i < dimension(); ++i) {
        m_components[i] += point[i];
    }

    return *this;
}

template <std::size_t Dimension>
point<Dimension> &point<Dimension>::operator-=(const point<Dimension> &point) {
    for(std::size_t i = 0u; i < dimension(); ++i) {
        m_components[i] -= point[i];
    }

    return *this;
}



template <std::size_t Dimension>
point<Dimension> &point<Dimension>::operator*=(distance distance) {
    for(std::size_t i = 0u; i < dimension(); ++i) {
        m_components[i] *= distance;
    }

    return *this;
}

template <std::size_t Dimension>
point<Dimension> &point<Dimension>::operator/=(distance distance) {
    for(std::size_t i = 0u; i < dimension(); ++i) {
        m_components[i] /= distance;
    }

    return *this;
}



template <std::size_t Dimension>
point<Dimension + 1u> push(const point<Dimension> &point_, distance distance) {
    point<Dimension + 1u> auxiliar;

    for(std::size_t i = 0u; i < Dimension; ++i) {
        auxiliar[i] = point_[i];
    }

    std::get<Dimension>(auxiliar) = distance;

    return auxiliar;
}

template <std::size_t Dimension>
point<Dimension - 1u> pop(const point<Dimension> &point_) {
    point<Dimension - 1u> auxiliar;

    for(std::size_t i = 0u; i < Dimension - 1u; ++i) {
        auxiliar[i] = point_[i];
    }

    return auxiliar;
}



namespace pnt {

template <typename Point>
Point at(const Point &point, utl::percentage percentage) {
    return point * static_cast<distance>(percentage);
}



template <typename Point>
distance length(const Point &point) {
    distance distance{ };

    for(std::size_t i = 0u; i < point.dimension(); ++i) {
        distance += point[i] * point[i];
    }

    return std::sqrt(distance);
}

template <typename Point>
distance separation(const Point &point_0, const Point &point_1) {
    return gmt::pnt::length(point_1 - point_0);
}



template <typename Point>
Point normal(const Point &point) {
    auto auxiliar = point;

    std::get<0u>(auxiliar) = std::get<1u>(point);
    std::get<1u>(auxiliar) = -std::get<0u>(point);

    return auxiliar;
}



template <typename... Point>
auto midpoint(const Point &... point) {
    return (point + ...) / static_cast<distance>(sizeof...(Point));
}



template <typename Point>
std::variant<utl::percentage, distance> cut(const Point &point, distance distance) {
    auto length_ = gmt::pnt::length(point);

    if(!length_ || distance > length_) {
        return static_cast<gmt::distance>(std::fma(distance, 1.f, -length_));
    }

    return static_cast<utl::percentage>(distance / length_);
}

}

}

#endif
