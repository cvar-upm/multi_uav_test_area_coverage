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

#if !defined(PLAN_AGENT_HEADER_IS_INCLUDED)
#define PLAN_AGENT_HEADER_IS_INCLUDED

#include "geometry.hpp"
#include "index.hpp"

namespace pln {

class agent {
public:
    agent(idx::index index, const gmt::point<3u> &home) : agent(index, home, home) { }

    agent(idx::index index, const gmt::point<3u> &position, const gmt::point<3u> &home) : m_index(index), m_position(position), m_home(home) { }

    idx::index index() const { return m_index; }

    gmt::point<3u> &position() { return m_position; }

    const gmt::point<3u> &position() const { return m_position; }

    gmt::point<3u> &home() { return m_home; }

    const gmt::point<3u> &home() const { return m_home; }

private:
    idx::index m_index;
    gmt::point<3u> m_position;
    gmt::point<3u> m_home;
};



inline std::ostream &operator<<(std::ostream &stream, const agent &agent) {
    return stream << '{' << agent.index() << ',' << ' ' << agent.position() << ',' << ' ' << agent.home() << '}';
}

}

#endif
