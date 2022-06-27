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
 * "AS IS" AND ANY EChainForwardPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EChainForwardEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#if !defined(INDEX_CHAIN_HEADER_IS_INCLUDED)
#define INDEX_CHAIN_HEADER_IS_INCLUDED

#include "geometry.hpp"

#include "index/index.hpp"

namespace idx {

template <typename Sequence>
class chain {
public:
    using sequence_type = Sequence;
    using point_type = typename gmt::chain<sequence_type>::point_type;
    using value_type = typename gmt::chain<sequence_type>::value_type;
    using continuum_type = typename gmt::chain<sequence_type>::continuum_type;

    template <typename ChainForward>
    chain(index index, ChainForward &&chain) : m_index(index), m_chain(std::forward<ChainForward>(chain)) { }

    index getindex() const { return m_index; }

    gmt::chain<sequence_type> &getchain() { return m_chain; }

    const gmt::chain<sequence_type> &getchain() const { return m_chain; }

    std::optional<value_type> operator*() const { return *m_chain; }

    chain &operator++();

    chain operator++(int);

    std::size_t size() const { return m_chain.size(); };

private:
    index m_index;
    gmt::chain<sequence_type> m_chain;
};



template <typename Chain>
chain<typename std::decay_t<Chain>::sequence_type> make_chain(index index, Chain &&thechain) {
    return chain<typename std::decay_t<Chain>::sequence_type>(index, std::forward<Chain>(thechain));
}



template <typename Sequence>
auto operator+(const chain<Sequence> &chain_, const typename chain<Sequence>::value_type &value) {
    return idx::make_chain(chain_.getindex(), chain_.chain() + value);
}

template <typename Sequence>
auto operator+(const typename chain<Sequence>::value_type &value, const chain<Sequence> &chain) {
    return idx::make_chain(chain.getindex(), value + chain.chain());
}



template <typename Sequence>
std::ostream &operator<<(std::ostream &stream, const chain<Sequence> &chain) {
    return stream << '<' << chain.getindex() << ',' << ' ' << chain.getchain() << '>';
}



template <typename Sequence>
chain<Sequence> &chain<Sequence>::operator++() {
    ++m_chain;

    return *this;
}

template <typename Sequence>
chain<Sequence> chain<Sequence>::operator++(int) {
    auto auxiliar = *this;

    operator++();

    return auxiliar;
}



template <typename Sequence>
auto pop(const chain<Sequence> &chain) {
    return idx::make_chain(chain.getindex(), gmt::pop(chain.getchain()));
}

}

#endif
