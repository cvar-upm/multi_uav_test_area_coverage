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

#include <iostream>

#include "plan.hpp"

#include "mutac_msgs/GeneratePlan.h"
#include "mutac_msgs/UpdatePlan.h"

#include "ros/ros.h"

auto get_homeset(const ros::NodeHandle &handle) {
    int size;

    if(!handle.getParam("n_drones", size)) {
        throw std::logic_error("Missing Parameter" ":" " " "n_drones");
    }

    std::vector<gmt::point<3u>> home_vector;

    auto lambda = [&handle](auto index) {
        std::vector<float> components(3u);

        auto parameter = "homebase/drone" + std::to_string(index + 1);

        if(!handle.getParam(parameter, components)) {
            throw std::logic_error("Missing Parameter" ":" " " + parameter);
        }

        return gmt::point<3u>(components[0u], components[1u], components[2u]);
    };

    auto sequence = seq::transform(lambda, seq::make_numeric_sequence(size));

    return seq::make_container_sequence_(sequence);
}

pln::parameters get_parameters(const ros::NodeHandle &handle) {
    float common_space_height;
    float common_space_increment;

    if(!handle.getParam("common_space/height", common_space_height)) {
        throw std::logic_error("Missing Parameter" ":" " " "common_space/height");
    }

    if(!handle.getParam("common_space/height_increment", common_space_increment)) {
        throw std::logic_error("Missing Parameter" ":" " " "common_space/height_increment");
    }

    float mission_height;
    float mission_distance;

    if(!handle.getParam("mission/height", mission_height)) {
        throw std::logic_error("Missing Parameter" ":" " " "mission/height");
    }

    if(!handle.getParam("mission/distance", mission_distance)) {
        throw std::logic_error("Missing Parameter" ":" " " "mission/distance");
    }

    return pln::parameters(mission_distance, common_space_height, common_space_increment, mission_height);
}

float get_threshold(const ros::NodeHandle &handle) {
    float threshold;

    if(!handle.getParam("update_plan/threshold", threshold)) {
        throw std::logic_error("Missing Parameter" ":" " " "update_plan/threshold");
    }

    return threshold;
}



gmt::point<2u> as_point_2(const geometry_msgs::Point32 &point) {
    return gmt::point<2u>(point.x, point.y);
}

gmt::point<2u> as_point_2(const geometry_msgs::Point &point) {
    return gmt::point<2u>(static_cast<float>(point.x), static_cast<float>(point.y));
}

gmt::point<3u> as_point_3(const geometry_msgs::Point &point) {
    return gmt::point<3u>(static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z));
}



auto as_polygon(const geometry_msgs::Polygon &polygon) {
    auto lambda = [](const auto &point) {
        return as_point_2(point);
    };

    auto sequence = seq::make_container_sequence(polygon.points);

    return gmt::make_polygon(gmt::make_chain(seq::transform(lambda, std::move(sequence))));
}

auto as_region(const mutac_msgs::Sweep &sweep) {
    auto polygon = as_polygon(sweep.polygon);

    auto direction = as_point_2(sweep.orientation);

    return pln::make_region(std::move(polygon), direction);
}

auto as_regionset(const mutac_msgs::Generation &generation) {
    auto lambda = [](const auto &sweep) {
        return as_region(sweep);
    };

    return seq::transform(lambda, seq::make_container_sequence(generation.sweeps));
}



idx::index as_index(mutac_msgs::Identifier identifier) {
    return static_cast<idx::index>(identifier.natural);
}

lbl::label as_label(mutac_msgs::Label label) {
    switch(label.natural) {
        case mutac_msgs::Label::POSITIONING_LABEL: {
            return false;
        }

        case mutac_msgs::Label::COVERING_LABEL: {
            return true;
        }

        default: {
            throw std::logic_error(__func__);
        }
    }
}

lbl::point<gmt::point<3u>> as_labeled_point_3(const mutac_msgs::LabeledPoint &point) {
    return lbl::make_point(as_label(point.label), as_point_3(point.point));
}

auto as_path(const mutac_msgs::LabeledPath &path) {
    auto lambda = [](const auto &point) {
        return as_labeled_point_3(point);
    };

    auto sequence = seq::make_container_sequence(path.points);

    auto chain = gmt::make_chain(seq::transform(lambda, std::move(sequence)));

    return idx::make_chain(as_index(path.identifier), std::move(chain));
}

auto as_plan(const mutac_msgs::Plan &plan) {
    auto lambda = [](const auto &path) {
        return as_path(path);
    };

    return gmt::make_scrawl(seq::transform(lambda, seq::make_container_sequence(plan.paths)));
}



mutac_msgs::Identifier to_identifier(idx::index index) {
    mutac_msgs::Identifier identifier;

    identifier.natural = static_cast<decltype(identifier.natural)>(index);

    return identifier;
}

mutac_msgs::Label to_label(lbl::label label) {
    mutac_msgs::Label result;

    result.natural = label ? mutac_msgs::Label::COVERING_LABEL : mutac_msgs::Label::POSITIONING_LABEL;

    return result;
}

template <typename Point>
geometry_msgs::Point to_point(const Point &point) {
    geometry_msgs::Point result;

    result.x = std::get<0u>(point);
    result.y = std::get<1u>(point);
    result.z = std::get<2u>(point);

    return result;
}

template <typename Point>
mutac_msgs::LabeledPoint to_labeled_point(const Point &point) {
    mutac_msgs::LabeledPoint result;

    result.label = to_label(point.getlabel());
    result.point = to_point(point);

    return result;
}

template <typename Path>
mutac_msgs::LabeledPath to_labeled_path(Path &&path) {
    mutac_msgs::LabeledPath result;

    result.identifier = to_identifier(path.getindex());

    auto lambda = [](const auto &point) {
        return to_labeled_point(point);
    };

    result.points = seq::accumulate<std::vector<mutac_msgs::LabeledPoint>>(seq::transform(lambda, std::forward<Path>(path)));

    return std::move(result);
}

template <typename Plan>
mutac_msgs::Plan to_plan(Plan &&plan) {
    mutac_msgs::Plan result;

    auto lambda = [](auto &&path) {
        return to_labeled_path(std::forward<decltype(path)>(path));
    };

    result.paths = seq::accumulate<std::vector<mutac_msgs::LabeledPath>>(seq::transform(lambda, std::forward<Plan>(plan)));

    return std::move(result);
}



template <typename Homeset>
class generate_plan_service {
public:
    using homeset_type = Homeset;

    template <typename HomesetForward>
    generate_plan_service(HomesetForward &&homeset, const pln::parameters *parameters, const ros::Publisher *publisher): m_homeset(std::forward<HomesetForward>(homeset)), m_parameters(parameters), m_publisher(publisher) { }

    bool callback(mutac_msgs::GeneratePlan::Request &request, mutac_msgs::GeneratePlan::Response &response);

private:
    homeset_type m_homeset;
    const pln::parameters *m_parameters;
    const ros::Publisher *m_publisher;
};

template <typename Homeset>
generate_plan_service<std::decay_t<Homeset>> make_generate_plan_service(Homeset &&homeset, const pln::parameters *parameters, const ros::Publisher *publisher) {
    return generate_plan_service<std::decay_t<Homeset>>(std::forward<Homeset>(homeset), parameters, publisher);
}

template <typename Homeset>
bool generate_plan_service<Homeset>::callback(mutac_msgs::GeneratePlan::Request &request, mutac_msgs::GeneratePlan::Response &response) {
    std::cout << "Generate Plan Invoked." << '\n';

    try {
        auto plan = pln::generate_plan(as_regionset(request.generation), m_homeset, *m_parameters);

        m_publisher->publish(to_plan(std::move(plan)));

        std::cout << "Plan Generated and Published." << '\n';
    } catch(const std::exception &) {
        std::cout << "Not Possible to Create Plan." << '\n';
    }

    std::cout << std::endl;

    return true;
}



class update_plan_service {
public:
    update_plan_service(const pln::parameters *parameters, float threshold, const ros::Publisher *publisher): m_parameters(parameters), m_threshold(threshold), m_publisher(publisher) { }

    bool callback(mutac_msgs::UpdatePlan::Request &request, mutac_msgs::UpdatePlan::Response &response);

private:
    const pln::parameters *m_parameters;
    float m_threshold;
    const ros::Publisher *m_publisher;
};

bool update_plan_service::callback(mutac_msgs::UpdatePlan::Request &request, mutac_msgs::UpdatePlan::Response &response) {
    std::cout << "Update Plan Invoked." << '\n';

    try {
        if(auto optional = pln::update_plan(as_plan(request.plan), *m_parameters, m_threshold)) {
            m_publisher->publish(to_plan(*std::move(optional)));

            std::cout << "New Plan Generated and Published." << '\n';
        } else {
            std::cout << "New Plan Not Worth." << '\n';
            std::cout << "Plan Update Skipped." << '\n';
        }
    } catch(const std::exception &) {
        std::cout << "Not Possible to Create New Plan." << '\n';
        std::cout << "Plan Update Skipped." << '\n';
    }

    std::cout << std::endl;

    return true;
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "planner");

    ros::NodeHandle handle;

    std::cout << std::endl;

    auto homeset = get_homeset(handle);

    auto parameters = get_parameters(handle);

    auto threshold = get_threshold(handle);

    std::cout << "Homeset" << ':' << ' ' << homeset << '\n';

    std::cout << "Parameters" << ':' << '\n';
    std::cout << '\t' << "Separation" << ':' << ' ' << parameters.separation() << '\n';
    std::cout << '\t' << "Base" << ':' << ' ' << parameters.base() << '\n';
    std::cout << '\t' << "Increment" << ':' << ' ' << parameters.increment() << '\n';
    std::cout << '\t' << "Inspection" << ':' << ' ' << parameters.inspection() << '\n';

    std::cout << "Threshold" << ':' << ' ' << threshold << '\n';

    std::cout << std::endl;



    auto planned_paths_publisher = handle.advertise<mutac_msgs::Plan>("planned_paths", 16u);



    auto generate_plan_service = make_generate_plan_service(homeset.lightweight(), &parameters, &planned_paths_publisher);

    auto generate_plan_server = handle.advertiseService("generate_plan", &decltype(generate_plan_service)::callback, &generate_plan_service);



    update_plan_service update_plan_service(&parameters, threshold, &planned_paths_publisher);

    auto update_plan_server = handle.advertiseService("update_plan", &decltype(update_plan_service)::callback, &update_plan_service);



    ros::spin();

    return 0;
}
