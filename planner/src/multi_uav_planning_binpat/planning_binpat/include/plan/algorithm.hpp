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

#if !defined(PLAN_ALGORITHM_HEADER_IS_INCLUDED)
#define PLAN_ALGORITHM_HEADER_IS_INCLUDED

#include "assignment.hpp"
#include "label.hpp"

#include "plan/agent.hpp"
#include "plan/parameters.hpp"
#include "plan/region.hpp"

namespace pln {

template <typename Plan>
auto agents(Plan &&plan) {
    auto lambda = [](auto &&path) {
        auto index = path.getindex();

        auto front = seq::front(path);

        return agent(index, front, seq::back(std::forward<decltype(path)>(path)));
    };

    return seq::transform(lambda, std::forward<Plan>(plan));
}

template <typename Plan>
auto availiable(Plan &&plan) {
    return pln::agents(idx::scrwl::nonnull(std::forward<Plan>(plan)));
}



template <typename Path>
auto divide(Path &&path, std::size_t size, gmt::distance separation) {
    return lbl::scrwl::trim(gmt::scrwl::splits(std::forward<Path>(path), size, separation));
}



template <typename Pathset, typename Swarm>
auto matrix(Pathset &&pathset, Swarm &&swarm) {
    auto lambda = [](auto &&path, const auto &agent) {
        auto position = pop(agent.position());

        auto home = pop(agent.home());

        auto front = seq::front(path);

        auto cost = gmt::pnt::separation(position, front.getpoint()) + gmt::pnt::separation(home, seq::back(std::forward<decltype(path)>(path)).getpoint());

        return cost;
    };

    return seq::confront(std::forward<Pathset>(pathset), std::forward<Swarm>(swarm), lambda);
}

template <typename Distanceset>
auto heights(Distanceset &&distanceset, gmt::distance base, gmt::distance increment) {
    auto lambda = [base, increment, size = seq::size(distanceset)](auto index) {
        return base + increment * (size - index - 1u);
    };

    return seq::transform(lambda, seq::sort(std::forward<Distanceset>(distanceset)));
}

template <typename Pathset, typename Swarm>
auto assign(Pathset &&pathset, Swarm &&swarm, gmt::distance base, gmt::distance increment) {
    auto matrix = seq::make_container_sequence_(pln::matrix(std::forward<Pathset>(pathset), swarm));

    auto assignment = assgn::jonker_volgenant(matrix.lightweight());

    auto new_swarm = seq::arrange(std::forward<Swarm>(swarm), assignment);

    auto lambda = [](const auto &agent, auto height) {
        return std::pair(agent, height);
    };

    auto heights = seq::transform(lambda, std::move(new_swarm), pln::heights(seq::deref(matrix.lightweight(), std::move(assignment)), base, increment));

    return seq::make_cached_sequence(std::move(matrix).container(), std::move(heights));
}



template <typename Chain>
auto levitate(Chain &&chain, gmt::distance height) {
    auto lambda = [height](const auto &point_0, const auto &point_1) {
        auto lambda_0 = [point_1] {
            return seq::lift(point_1);
        };

        auto lambda_1 = [point_0, point_1, height] {
            auto first = lbl::falset(point_0);

            std::get<2u>(first) = height;

            auto second = lbl::falset(point_1);

            std::get<2u>(second) = height;

            return seq::lift(first) + second + point_1;
        };

        auto boolean = std::get<2u>(point_0) == std::get<2u>(point_1) || pop(point_0) == pop(point_1);

        return seq::conditional(boolean, lambda_0, lambda_1);
    };

    auto front = seq::front(chain);

    auto next = seq::next(chain);

    return front + seq::and_then(lambda, std::forward<Chain>(chain), std::move(next));
}

template <typename Path>
auto guide(Path &&path, const agent &agent, gmt::distance positioning, gmt::distance inspection) {
    auto lambda = [positioning, inspection](const auto &point) {
        return point.getlabel() ? inspection : positioning;
    };

    auto closest = gmt::chn::closest(path, lbl::make_point(false, pop(agent.position())));

    auto raw_chain = gmt::chn::push(std::forward<Path>(path), lambda);

    auto chain = lbl::make_point(false, agent.position()) + std::move(raw_chain) + lbl::make_point(false, agent.home());

    auto levitate = seq::make_container_sequence_(pln::levitate(gmt::make_chain(chain), positioning));

    auto nonempty = seq::make_container_sequence_(seq::nonrepeated(levitate.lightweight()));

    return idx::make_chain(agent.index(), gmt::make_chain(std::move(nonempty)));
}

template <typename Pathset, typename Assignment>
auto decorate(Pathset &&pathset, Assignment &&assignment, gmt::distance inspection) {
    auto lambda = [inspection](auto &&path, const auto &pair) {
        return pln::guide(std::forward<decltype(path)>(path), pair.first, pair.second, inspection);
    };

    return gmt::make_scrawl(seq::transform(lambda, std::forward<Pathset>(pathset), std::forward<Assignment>(assignment)));
}



template <typename Plan>
bool select(Plan &&plan, idx::weight weight, float threshold) {
    return idx::scrwl::getweight(std::forward<Plan>(plan)) / weight <= threshold;
}



template <typename Path, typename Swarm>
auto common(Path &&path, Swarm &&swarm, const pln::parameters &parameters, float threshold = 1.f, idx::weight weight = std::numeric_limits<gmt::distance>::infinity()) {
    auto pathset = seq::make_container_sequence_(pln::divide(std::forward<Path>(path), seq::size(swarm), parameters.separation()));

    auto assignment = pln::assign(pathset.lightweight(), std::forward<Swarm>(swarm), parameters.base(), parameters.increment());

    auto plan = seq::make_container_sequence_(pln::decorate(pathset.lightweight(), std::move(assignment), parameters.inspection()));

    auto selection = pln::select(plan.lightweight(), weight, threshold);

    auto result = gmt::make_scrawl(std::move(plan));

    if(selection) {
        return std::make_optional(std::move(result));
    } else {
        return std::optional<decltype(result)>();
    }
}



template <typename Polygon>
auto back_and_forth(Polygon &&polygon, const typename std::decay_t<Polygon>::point_type &orientation) {
    auto front = seq::front(polygon);

    return seq::alternate(gmt::plygn::intersections(seq::as_sequence, std::forward<Polygon>(polygon), gmt::parallels(gmt::make_segment(front, front + gmt::pnt::normal(orientation)), orientation)));
}

template <typename Polygon>
auto shortest_sweep(Polygon &&polygon, const typename std::decay_t<Polygon>::point_type &orientation, const typename std::decay_t<Polygon>::point_type &point) {
    auto sweeps = seq::make_container_sequence_(pln::back_and_forth(std::forward<Polygon>(polygon), orientation));

    auto closest = gmt::bchn::closest(sweeps.lightweight(), point);

    return gmt::make_chain(seq::plop(std::move(sweeps), closest));
}

template <typename Regionset, typename Point>
auto generate(Regionset &&regionset, const Point &point) {
    auto lambda_0 = [point](auto &&region) {
        return std::make_optional(seq::make_container_sequence_(pln::shortest_sweep(std::forward<decltype(region)>(region).polygon(), region.direction(), point)));
    };

    auto lambda_1 = [](auto &&chain, auto &&region) {
        return std::make_optional(seq::make_container_sequence_(pln::shortest_sweep(std::forward<decltype(region)>(region).polygon(), region.direction(), seq::back(std::forward<decltype(chain)>(chain)))));
    };

    auto sequence = seq::make_container_sequence_(seq::make_while_sequence(lambda_0, lambda_1, std::forward<Regionset>(regionset)));

    return lbl::scrwl::link(lbl::scrwl::make_scrawl(std::move(sequence)));
}



template <typename Homeset>
auto get_swarm(Homeset &&homeset) {
    auto lambda = [](const auto &point, auto index) {
        return agent(index, point, point);
    };

    return seq::transform(lambda, std::forward<Homeset>(homeset), seq::make_numeric_sequence<std::size_t>());
}

template <typename Regionset, typename Homeset>
auto generate_plan(Regionset &&regionset, Homeset &&homeset, const parameters &parameters) {
    auto path = seq::make_container_sequence_(pln::generate(std::forward<Regionset>(regionset), pop(gmt::chn::midpoint(homeset))));

    auto agents = seq::make_container_sequence_(pln::get_swarm(std::forward<Homeset>(homeset)));

    return pln::common(gmt::make_chain(path.lightweight()), agents.lightweight(), parameters).value();
}



template <typename Plan>
auto link(Plan &&plan) {
    auto lambda_0 = [](const auto &chain) {
        return !seq::empty(chain);
    };

    auto lambda_1 = [](auto &&chain) {
        return seq::make_container_sequence_(std::forward<decltype(chain)>(chain));
    };

    auto scrawl = seq::make_container_sequence_(seq::transform(lambda_1, seq::filter(lbl::scrwl::trim(pop(std::forward<Plan>(plan))), lambda_0)));

    auto lambda_2 = [&scrawl](auto index) {
        return gmt::make_chain(scrawl.container()[index].lightweight());
    };

    auto sequence = seq::make_container_sequence_(seq::transform(lambda_2, seq::make_index_sequence(scrawl)));

    return seq::make_container_sequence_(lbl::scrwl::link(gmt::make_scrawl(std::move(sequence))));
}



template <typename Plan>
auto update_plan(Plan &&plan, const parameters &parameters, float threshold) {
    auto swarm = seq::make_container_sequence_(pln::availiable(plan));

    auto weight = idx::scrwl::getweight(plan);

    auto path = pln::link(std::forward<Plan>(plan));

    return pln::common(gmt::make_chain(path.lightweight()), swarm.lightweight(), parameters, threshold, weight);
}



template <typename Polygon, typename Homeset>
auto planning_binpat(const region<Polygon> &region, Homeset &&homeset, const parameters &parameters) {
    auto path = seq::make_container_sequence_(pln::shortest_sweep(region.polygon(), region.direction(), pop(gmt::chn::midpoint(homeset))));

    auto agents = seq::make_container_sequence_(pln::get_swarm(std::forward<Homeset>(homeset)));

    return pln::common(lbl::chn::make_chain(gmt::make_chain(path.lightweight())), agents.lightweight(), parameters).value();
}

}

#endif
