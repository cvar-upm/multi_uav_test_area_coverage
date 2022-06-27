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

#if !defined(GEOMETRY_POLYGON_HEADER_IS_INCLUDED)
#define GEOMETRY_POLYGON_HEADER_IS_INCLUDED

#include "geometry/chain.hpp"

namespace gmt {

template <typename Sequence>
class polygon {
public:
    using sequence_type = Sequence;
    using point_type = typename sequence_type::value_type;
    using value_type = point_type;
    using continuum_type = segment<point_type>;

    template <typename ChainForward>
    polygon(utl::forward_t, ChainForward &&chain) : m_chain(std::forward<ChainForward>(chain)) { }

    gmt::chain<sequence_type> &chain() { return m_chain; }

    const gmt::chain<sequence_type> &chain() const { return m_chain; }

    std::optional<value_type> operator*() const { return *m_chain; }

    polygon &operator++();

    polygon operator++(int);

    std::size_t size() const { return m_chain.size(); };

private:
    gmt::chain<sequence_type> m_chain;
};



template <typename Sequence>
polygon<Sequence> &polygon<Sequence>::operator++() {
    ++m_chain;

    return *this;
}

template <typename Sequence>
polygon<Sequence> polygon<Sequence>::operator++(int) {
    auto auxiliar = *this;

    operator++();

    return auxiliar;
}



template <typename Chain>
polygon<typename std::decay_t<Chain>::sequence_type> make_polygon(Chain &&chain) {
    return polygon<typename std::decay_t<Chain>::sequence_type>(utl::forward, std::forward<Chain>(chain));
}



template <typename Sequence>
std::ostream &operator<<(std::ostream &stream, const polygon<Sequence> &polygon) {
    return stream << polygon.chain();
}



template <typename Sequence>
auto vertices(const polygon<Sequence> &polygon) {
    return polygon.chain() + seq::front(polygon);
}



namespace plygn {

template <typename Polygon>
std::optional<std::pair<seq::continuous_size, seq::continuous_size>> intersect(Polygon &&polygon, const line<typename std::decay_t<Polygon>::point_type> &line) {
    auto lambda = [polygon](const auto &size_0, const auto size_1) {
        return std::pair(size_0, seq::continuous_size(seq::size(polygon) - size_1.first - 1u, 1 - size_1.second));
    };

    auto points = gmt::vertices(std::forward<Polygon>(polygon));

    return utl::opt::transform(lambda, gmt::chn::intersect(points, line), gmt::chn::intersect(gmt::make_chain(seq::reverse(points)), line));
}



template <typename Polygon, typename Lineset>
auto intersections(Polygon &&polygon, Lineset &&lineset) {
    auto lambda = [polygon = std::forward<Polygon>(polygon)](const auto &line) {
        return gmt::plygn::intersect(polygon, line);
    };

    return seq::make_while_sequence_(lambda, std::forward<Lineset>(lineset));
}

template <typename Polygon, typename Lineset>
auto intersections(seq::as_sequence_t, Polygon &&polygon, Lineset &&lineset) {
    auto lambda = [chain = vertices(polygon)](const auto &pair) {
        return std::pair(seq::at(chain, pair.first), seq::at(chain, pair.second));
    };

    return seq::transform(std::move(lambda), gmt::plygn::intersections(std::forward<Polygon>(polygon), std::forward<Lineset>(lineset)));
}

}

}

#endif
