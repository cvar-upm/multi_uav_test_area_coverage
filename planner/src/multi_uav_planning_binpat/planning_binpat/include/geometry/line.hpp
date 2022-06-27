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

#if !defined(GEOMETRY_LINE_HEADER_IS_INCLUDED)
#define GEOMETRY_LINE_HEADER_IS_INCLUDED

#include "geometry/point.hpp"

namespace gmt {

template <typename Point>
class line {
public:
    using point_type = Point;

    line(const point_type &point_0, const point_type &point_1) : m_a(std::get<1u>(point_1) - std::get<1u>(point_0)), m_b(std::get<0u>(point_0) - std::get<0u>(point_1)), m_c(m_a * std::get<0u>(point_0) + m_b * std::get<1u>(point_0)), m_point(point_0) { }

    distance a() const { return m_a; }

    distance b() const { return m_b; }

    surface c() const { return m_c; }

    point_type point() const { return m_point; }

private:
    distance m_a;
    distance m_b;
    surface m_c;
    point_type m_point;
};



template <typename Point>
std::optional<Point> intersect(const line<Point> &line_0, const line<Point> &line_1) {
    auto determinant = line_0.a() * line_1.b() - line_1.a() * line_0.b();

    if(determinant == surface{ }) {
        return { };
    }

    auto x = (line_1.b() * line_0.c() - line_0.b() * line_1.c()) / determinant;
    auto y = (line_0.a() * line_1.c() - line_1.a() * line_0.c()) / determinant;

    auto auxiliar = line_0.point();

    std::get<0u>(auxiliar) = x;
    std::get<1u>(auxiliar) = y;

    return auxiliar;
}

}

#endif
