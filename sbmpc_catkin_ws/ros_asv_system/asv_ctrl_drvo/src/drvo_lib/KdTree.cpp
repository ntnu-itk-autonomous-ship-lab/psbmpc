/**
 * \file   KdTree.cpp
 * \brief  Defines the KdTree class.
 */

#ifndef DRVO_KD_TREE_H_
#include "KdTree.h"
#endif

#include <algorithm>
#include <limits>

#ifndef DRVO_AGENT_H_
#include "drvoAgent.h"
#endif
#include "iostream" // for test printout!


	//KdTree::KdTree(){ }
	//KdTree::KdTree(Simulator *simulator) : simulator_(simulator) { }
	KdTree::KdTree(Agent *agent) : agent_(agent) { }

	void KdTree::build()
	{
		
		if(obstacles_.size() > agent_->obstacles_.size()){
			obstacles_.resize(agent_->obstacles_.size());
		}else{
			obstacles_.reserve(agent_->obstacles_.size());
		}
			
		//std::cout << "agent_->obstacles_ : " << agent_->obstacles_.size() << std::endl; //
		//std::cout << "kdTree obstacles_ : " << obstacles_.size() << std::endl; // check test!

		
		for (std::size_t i = obstacles_.size(); i < agent_->obstacles_.size(); ++i) {
			obstacles_.push_back(i); // obstacle numbering from zero
		}

		//std::cout << "KDtree obstacles_ : " << obstacles_.size() << std::endl; // check test!
		
		
		nodes_.resize(2 * agent_->obstacles_.size() - 1); // -1 needed when using only obstacles? 
		

		if (!obstacles_.empty()) {
			buildRecursive(0, obstacles_.size(), 0);
		}
	}

	void KdTree::buildRecursive(std::size_t begin, std::size_t end, std::size_t node)
	{
		nodes_[node].begin_ = begin;
		nodes_[node].end_ = end;
		nodes_[node].minX_ = nodes_[node].maxX_ = agent_->obstacles_[obstacles_[begin]]->position_.getX();  // is this the same as obst_vect[k]->x_[0]? YES!
		nodes_[node].minY_ = nodes_[node].maxY_ = agent_->obstacles_[obstacles_[begin]]->position_.getY();  // same as actual obstacle Y-position!

		for (std::size_t i = begin + 1; i < end; ++i) {
			if (agent_->obstacles_[obstacles_[i]]->position_.getX() > nodes_[node].maxX_) {
				nodes_[node].maxX_ = agent_->obstacles_[obstacles_[i]]->position_.getX();
			}
			else if (agent_->obstacles_[obstacles_[i]]->position_.getX() < nodes_[node].minX_) {
				nodes_[node].minX_ = agent_->obstacles_[obstacles_[i]]->position_.getX();
			}

			if (agent_->obstacles_[obstacles_[i]]->position_.getY() > nodes_[node].maxY_) {
				nodes_[node].maxY_ = agent_->obstacles_[obstacles_[i]]->position_.getY();
			}
			else if (agent_->obstacles_[obstacles_[i]]->position_.getY() < nodes_[node].minY_) {
				nodes_[node].minY_ = agent_->obstacles_[obstacles_[i]]->position_.getY();
			}
		}

		if (end - begin > MAX_LEAF_SIZE) {
			const bool vertical = nodes_[node].maxX_ - nodes_[node].minX_ > nodes_[node].maxY_ - nodes_[node].minY_;
			const float split = 0.5f * (vertical ?  nodes_[node].maxX_ + nodes_[node].minX_ : nodes_[node].maxY_ + nodes_[node].minY_);

			std::size_t left = begin;
			std::size_t right = end - 1;

			while (true) {
				while (left <= right && (vertical ? agent_->obstacles_[obstacles_[left]]->position_.getX()
										 : agent_->obstacles_[obstacles_[left]]->position_.getY()) < split) {
					++left;
				}

				while (right >= left && (vertical ? agent_->obstacles_[obstacles_[right]]->position_.getX()
										 : agent_->obstacles_[obstacles_[right]]->position_.getY()) >= split) {
					--right;
				}

				if (left > right) {
					break;
				}
				else {
					std::swap(obstacles_[left], obstacles_[right]);
					++left;
					--right;
				}
			}

			if (left == begin) {
				++left;
				++right;
			}

			nodes_[node].left_ = node + 1;
			nodes_[node].right_ = 2 * (left - begin) + node;

			buildRecursive(begin, left, nodes_[node].left_);
			buildRecursive(left, end, nodes_[node].right_);
		}
	}

	void KdTree::queryRecursive(Agent *agent, float &rangeSq, std::size_t node) const
	{
		if (nodes_[node].end_ - nodes_[node].begin_ <= MAX_LEAF_SIZE) {
			for (std::size_t i = nodes_[node].begin_; i < nodes_[node].end_; ++i) {
				agent->insertNeighbor(obstacles_[i], rangeSq);
			}
		}
		else {
			float distSqLeft = 0.0f;
			float distSqRight = 0.0f;
			float distDiff = 0.0f;

			if (agent->position_.getX() < nodes_[nodes_[node].left_].minX_) {
				distDiff = nodes_[nodes_[node].left_].minX_ - agent->position_.getX();
				distSqLeft += (distDiff*distDiff);
			}
			else if (agent->position_.getX() > nodes_[nodes_[node].left_].maxX_) {
				//distSqLeft += sqr(agent->position_.getX() - nodes_[nodes_[node].left_].maxX_);
				distDiff = agent->position_.getX() - nodes_[nodes_[node].left_].maxX_;
				distSqLeft += (distDiff*distDiff);
			}

			if (agent->position_.getY() < nodes_[nodes_[node].left_].minY_) {
				//distSqLeft += sqr(nodes_[nodes_[node].left_].minY_ - agent->position_.getY());
				distDiff = nodes_[nodes_[node].left_].minY_ - agent->position_.getY();
				distSqLeft += (distDiff*distDiff);
			}
			else if (agent->position_.getY() > nodes_[nodes_[node].left_].maxY_) {
				//distSqLeft += sqr(agent->position_.getY() - nodes_[nodes_[node].left_].maxY_);
				distDiff = agent->position_.getY() - nodes_[nodes_[node].left_].maxY_;
				distSqLeft += (distDiff*distDiff);
			}

			if (agent->position_.getX() < nodes_[nodes_[node].right_].minX_) {
				//distSqRight += sqr(nodes_[nodes_[node].right_].minX_ - agent->position_.getX());
				distDiff = nodes_[nodes_[node].right_].minX_ - agent->position_.getX();
				distSqLeft += (distDiff*distDiff);
			}
			else if (agent->position_.getX() > nodes_[nodes_[node].right_].maxX_) {
				//distSqRight += sqr(agent->position_.getX() - nodes_[nodes_[node].right_].maxX_);
				distDiff = agent->position_.getX() - nodes_[nodes_[node].right_].maxX_;
				distSqLeft += (distDiff*distDiff);
			}

			if (agent->position_.getY() < nodes_[nodes_[node].right_].minY_) {
				//distSqRight += sqr(nodes_[nodes_[node].right_].minY_ - agent->position_.getY());
				distDiff = nodes_[nodes_[node].right_].minY_ - agent->position_.getY();
				distSqLeft += (distDiff*distDiff);
			}
			else if (agent->position_.getY() > nodes_[nodes_[node].right_].maxY_) {
				//distSqRight += sqr(agent->position_.getY() - nodes_[nodes_[node].right_].maxY_);
				distDiff = agent->position_.getY() - nodes_[nodes_[node].right_].maxY_;
				distSqLeft += (distDiff*distDiff);
			}

			if (distSqLeft < distSqRight) {
				if (distSqLeft < rangeSq) {
					queryRecursive(agent, rangeSq, nodes_[node].left_);

					if (distSqRight < rangeSq) {
						queryRecursive(agent, rangeSq, nodes_[node].right_);
					}
				}
			}
			else {
				if (distSqRight < rangeSq) {
					queryRecursive(agent, rangeSq, nodes_[node].right_);

					if (distSqLeft < rangeSq) {
						queryRecursive(agent, rangeSq, nodes_[node].left_);
					}
				}
			}
		}
	}

