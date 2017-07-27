#pragma once

#include "link.hpp"
#include <vector>

namespace robo{

    class Chain{
    public:
        int nr_joints;
        int nr_links;
        std::vector<Link> links;

        Chain():nr_joints(0), nr_links(0){};

        Chain(const std::vector<Link>& links):nr_joints(0), nr_links(0){
            add_links(links);
        };
        
        void add_link(const Link& link){
            links.push_back(link);
            ++nr_links;
            if(link.has_joint()){
                ++nr_joints;
            }
        };

        void add_links(const std::vector<Link>& links){
            for(const auto& link : links){
                add_link(link);
            }
        };
    };

}
