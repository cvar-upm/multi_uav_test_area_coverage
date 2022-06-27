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

#if !defined(ASSIGNMENT_ALGORITHM_HEADER_IS_INCLUDED)
#define ASSIGNMENT_ALGORITHM_HEADER_IS_INCLUDED

#include <memory>

#include "sequence.hpp"

#include "assignment/bare/lap.h"

namespace assgn {

template <typename Matrix>
auto jonker_volgenant(Matrix &&matrix) {
    auto lambda = [](auto &&sequence) {
        return seq::accumulate<std::vector<cost>>(std::forward<decltype(sequence)>(sequence));
    };

    auto rows = seq::accumulate<std::vector<std::vector<cost>>>(seq::transform(lambda, std::forward<Matrix>(matrix)));

    std::vector<const cost *> pointers;

    pointers.reserve(rows.size());

    for(const auto &row : rows) {
        if(row.size() != rows.size()) {
            throw std::logic_error(__func__);
        }

        pointers.push_back(row.data());
    }

    std::vector<col> columns(rows.size());
    std::unique_ptr<row []> therows(new row[rows.size()]);
    std::unique_ptr<cost []> u(new cost[rows.size()]);
    std::unique_ptr<cost []> v(new cost[rows.size()]);

    lap(static_cast<int>(rows.size()), pointers.data(), columns.data(), therows.get(), u.get(), v.get());

    return seq::make_container_sequence(std::move(columns));
}

}

#endif
