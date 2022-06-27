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

#if !defined(PLAN_REGION_HEADER_IS_INCLUDED)
#define PLAN_REGION_HEADER_IS_INCLUDED

#include "geometry.hpp"

namespace pln {

template <typename Polygon>
class region {
public:
    using polygon_type = Polygon;

    template <typename PolygonForward>
    region(PolygonForward &&polygon, const gmt::point<2> &direction) : m_polygon(std::forward<PolygonForward>(polygon)), m_direction(direction) { }

    polygon_type &polygon() & { return m_polygon; }

    polygon_type &&polygon() && { return std::move(m_polygon); }

    const polygon_type &polygon() const & { return m_polygon; }

    gmt::point<2u> &direction() { return m_direction; }

    const gmt::point<2u> &direction() const { return m_direction; }

private:
    polygon_type m_polygon;
    gmt::point<2> m_direction;
};



template <typename Polygon>
region<std::decay_t<Polygon>> make_region(Polygon &&polygon, const gmt::point<2> &direction) {
    return region<std::decay_t<Polygon>>(std::forward<Polygon>(polygon), direction);
}



template <typename Polygon>
std::ostream &operator<<(std::ostream &stream, const region<Polygon> &region) {
    return stream << '{' << region.polygon() << ',' << ' ' << region.direction() << '}';
}

}

#endif
