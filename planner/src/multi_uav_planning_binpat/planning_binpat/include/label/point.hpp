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

#if !defined(LABEL_POINT_HEADER_IS_INCLUDED)
#define LABEL_POINT_HEADER_IS_INCLUDED

#include "geometry.hpp"

#include "label/label.hpp"

namespace lbl {

template <typename Point>
class point {
public:
    using point_type = Point;

    template <typename PointForward>
    point(label label, PointForward &&point) : m_label(label), m_point(std::forward<PointForward>(point)) { }

    label getlabel() const { return m_label; }

    point_type &getpoint() { return m_point; }

    const point_type &getpoint() const { return m_point; }

    std::size_t dimension() const { return getpoint().dimension(); }

    gmt::distance operator[](std::size_t index) { return m_point[index]; }

    gmt::distance operator[](std::size_t index) const { return m_point[index]; }

    point &operator+=(const point &point);

    point &operator-=(const point &point);

    point &operator*=(gmt::distance distance);

    point &operator/=(gmt::distance distance);

    operator point_type() const { return getpoint(); }

private:
    label m_label;
    point_type m_point;
};

}



namespace std {

template <typename Point>
struct tuple_size<lbl::point<Point>> : tuple_size<Point> { };



template <size_t Index, typename Point>
struct tuple_element<Index, lbl::point<Point>> {
    using type = tuple_element_t<Index, Point>;
};



template <size_t Index, typename Point>
gmt::distance &get(lbl::point<Point> &point) {
    return std::get<Index>(point.getpoint());
}

template <size_t Index, typename Point>
const gmt::distance &get(const lbl::point<Point> &point) {
    return std::get<Index>(point.getpoint());
}

}



namespace lbl {

template <std::size_t Index, typename Point>
gmt::distance &get(point<Point> &point) {
    return std::get<Index>(point.getpoint());
}

template <std::size_t Index, typename Point>
const gmt::distance &get(const point<Point> &point) {
    return std::get<Index>(point.getpoint());
}



template <typename Point>
point<std::decay_t<Point>> make_point(label label, Point &&thepoint) {
    return point<std::decay_t<Point>>(label, std::forward<Point>(thepoint));
}



template <typename Point>
point<Point> operator+(const point<Point> &point_0, const point<Point> &point_1) {
    auto auxiliar = point_0;

    auxiliar += point_1;

    return auxiliar;
}

template <typename Point>
point<Point> operator-(const point<Point> &point_0, const point<Point> &point_1) {
    auto auxiliar = point_0;

    auxiliar -= point_1;

    return auxiliar;
}



template <typename Point>
point<Point> operator*(const point<Point> &point_0, gmt::distance distance) {
    auto auxiliar = point_0;

    auxiliar *= distance;

    return auxiliar;
}

template <typename Point>
point<Point> operator*(gmt::distance distance, const point<Point> &point) {
    return point * distance;
}

template <typename Point>
point<Point> operator/(const point<Point> &point, gmt::distance distance) {
    auto auxiliar = point;

    auxiliar /= distance;

    return auxiliar;
}



template <typename Point>
bool operator==(const point<Point> &point_0, const point<Point> &point_1) {
    return point_0.getlabel() == point_1.getlabel() && point_0.getpoint() == point_1.getpoint();
}

template <typename Point>
bool operator!=(const point<Point> &point_0, const point<Point> &point_1) {
    return !(point_0 == point_1);
}



template <typename Point>
std::ostream &operator<<(std::ostream &stream, const point<Point> &point) {
    return stream << '(' << point.getlabel() << ',' << ' ' << point.getpoint() << ')';
}



template <typename Point>
point<Point> &point<Point>::operator+=(const point<Point> &point) {
    m_label = m_label && point.getlabel();

    m_point += point.getpoint();

    return *this;
}

template <typename Point>
point<Point> &point<Point>::operator-=(const point<Point> &point) {
    m_label = m_label && point.getlabel();

    m_point -= point.getpoint();

    return *this;
}



template <typename Point>
point<Point> &point<Point>::operator*=(gmt::distance distance) {
    m_point *= distance;

    return *this;
}

template <typename Point>
point<Point> &point<Point>::operator/=(gmt::distance distance) {
    m_point /= distance;

    return *this;
}



template <typename Point>
point<Point> falset(const point<Point> &point) {
    return lbl::make_point(false, point.getpoint());
}



template <typename Point>
auto push(const point<Point> &point, gmt::distance distance) {
    return lbl::make_point(point.getlabel(), push(point.getpoint(), distance));
}

template <typename Point>
auto pop(const point<Point> &point) {
    return lbl::make_point(point.getlabel(), pop(point.getpoint()));
}

}

#endif
