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

#if !defined(GEOMETRY_SEGMENT_HEADER_IS_INCLUDED)
#define GEOMETRY_SEGMENT_HEADER_IS_INCLUDED

#include "geometry/line.hpp"

namespace gmt {

template <typename Point>
class segment {
public:
    using point_type = Point;

    template <typename Endpoint0, typename Endpoint1>
    segment(Endpoint0 &&endpoint_0, Endpoint1 &&endpoint_1) : m_endpoints{std::forward<Endpoint0>(endpoint_0), std::forward<Endpoint1>(endpoint_1)} { }

    point_type &operator[](std::size_t index) { return m_endpoints[index]; }

    const point_type &operator[](std::size_t index) const { return m_endpoints[index]; }

    segment &operator+=(const point_type &point);

    segment &operator-=(const point_type &point);

private:
    std::array<point_type, 2u> m_endpoints;
};

}



namespace std {

template <typename Point>
struct tuple_size<gmt::segment<Point>> : std::integral_constant<size_t, 2u> { };



template <size_t Index, typename Point>
struct tuple_element<Index, gmt::segment<Point>> {
    using type = Point;
};



template <size_t Index, typename Point>
Point &get(gmt::segment<Point> &segment) {
    return segment[Index];
}

template <size_t Index, typename Point>
const Point &get(const gmt::segment<Point> &segment) {
    return segment[Index];
}

}



namespace gmt {

template <typename Point>
segment<Point> operator+(const segment<Point> &segment, const Point &point) {
    auto auxiliar = segment;

    auxiliar += point;

    return auxiliar;
}

template <typename Point>
segment<Point> operator+(const Point &point, const segment<Point> &segment) {
    return segment + point;
}

template <typename Point>
segment<Point> operator-(const segment<Point> &segment, const Point &point) {
    auto auxiliar = segment;

    auxiliar -= point;

    return auxiliar;
}



template <typename Point>
std::ostream &operator<<(std::ostream &stream, const segment<Point> &segment) {
    return stream << '{' << std::get<0u>(segment) << ',' << ' ' << std::get<1u>(segment) << '}';
}



template <typename Point>
segment<Point> &segment<Point>::operator+=(const point_type &point) {
    for(std::size_t i = 0u; i < 2u; ++i) {
        m_endpoints[i] += point;
    }

    return *this;
}

template <typename Point>
segment<Point> &segment<Point>::operator-=(const point_type &point) {
    for(std::size_t i = 0u; i < 2u; ++i) {
        m_endpoints[i] -= point;
    }

    return *this;
}



template <typename Endpoint0, typename Endpoint1>
segment<std::common_type_t<std::decay_t<Endpoint0>, std::decay_t<Endpoint1>>> make_segment(Endpoint0 &&endpoint_0, Endpoint1 &&endpoint_1) {
    return gmt::segment<std::common_type_t<std::decay_t<Endpoint0>, std::decay_t<Endpoint1>>>(std::forward<Endpoint0>(endpoint_0), std::forward<Endpoint1>(endpoint_1));
}



template <typename Point>
Point vector(const segment<Point> &segment) {
    return std::get<1u>(segment) - std::get<0u>(segment);
}



template <typename Point>
Point at(const segment<Point> &segment, utl::percentage percentage) {
    return std::get<0u>(segment) + gmt::pnt::at(gmt::vector(segment), percentage);
}



template <typename Point>
distance length(const segment<Point> &segment) {
    return gmt::pnt::length(gmt::vector(segment));
}



template <typename Point>
std::variant<utl::percentage, distance> cut(const segment<Point> &segment, distance length) {
    return gmt::pnt::cut(gmt::vector(segment), length);
}



template <typename Point>
std::optional<utl::percentage> intersect(const segment<Point> &segment, const line<Point> &line) {
    auto endpoint_0 = std::get<0u>(segment);
    auto endpoint_1 = std::get<1u>(segment);

    auto point = gmt::intersect(line, gmt::line(endpoint_0, endpoint_1));

    if(!point) {
        return { };
    }

    auto [min_x, max_x] = std::minmax(std::get<0u>(endpoint_0), std::get<0u>(endpoint_1));
    auto [min_y, max_y] = std::minmax(std::get<1u>(endpoint_0), std::get<1u>(endpoint_1));

    const auto &point_x = std::get<0u>(*point);
    const auto &point_y = std::get<1u>(*point);

    auto contains_x = min_x <= point_x && point_x <= max_x;
    auto contains_y = min_y <= point_y && point_y <= max_y;

    if(!contains_x || !contains_y) {
        return { };
    }

    return gmt::length(gmt::make_segment(endpoint_0, *point)) / gmt::length(segment);
}



template <typename Point>
auto parallels(const segment<Point> &segment, const Point &displacement) {
    auto lambda = [segment, displacement](auto index) {
        auto segment_ = segment + index * displacement;

        return gmt::line(std::get<0u>(segment_), std::get<1u>(segment_));
    };

    return seq::transform(lambda, seq::make_numeric_sequence<distance>());
}

}

#endif
