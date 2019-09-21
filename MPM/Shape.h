#pragma once
#include "Constants.h"

#include <vector>
#include <iostream>
#include <math.h>

namespace sdf {
	struct Shape {
	public:

		virtual real Sdf(vec2 p) = 0;

		real SdfRounded(vec2 p, real r) {
			return Sdf(p) - r;
		}

		real SdfHollow(vec2 p, real r1, real r2) {
			// r1 is inner rounding, r2 is outer rounding
			// r1 = 0 and r2 = 0 => the point cloud is empty
			// r1 can at most be the min distance to the center-line/skeleton of the shape
			return glm::max(-SdfRounded(p, -r1), SdfRounded(p, r2));
		}

		vec2 pos;
	};

	struct Circle : public Shape {
	private:
		Circle() {}

	public:
		Circle(vec2 _pos, real _r) {
			pos = _pos;
			r = _r;
		}

		real Sdf(vec2 p) {
			return glm::length(p - pos) - r;
		}

		real r;
	};

	struct Rectangle : public Shape {
	private:
		Rectangle() {}
	public:
		Rectangle(vec2 _pos, real _b, real _h) {
			pos = _pos;
			b = _b;
			h = _h;
		}

		real Sdf(vec2 p) {
			vec2 d = glm::abs(p - pos) - vec2(b/2.0, h/2.0);

			return glm::length(glm::max(d, vec2(0.0))) + glm::min(glm::max(d.x, d.y), 0.0);
		}

		real b, h; // b = horizontal base length, h = vertical height length;
	};

	struct IsoscelesTriangle : public Shape {
	private:
		IsoscelesTriangle() {}
	public:
		IsoscelesTriangle(vec2 _pos, real _b, real _h) {
			pos = _pos;
			b = _b;
			h = _h;
		}

		real Sdf(vec2 p) {
			p = p - pos;

			vec2 q = vec2(glm::abs(b/2.0), -glm::abs(h));
			p.x = abs(p.x);
			vec2 a = p - q * glm::clamp(glm::dot(p, q) / glm::dot(q, q), 0.0, 1.0);
			vec2 b = p - q * vec2(glm::clamp(p.x / q.x, 0.0, 1.0), 1.0);
			real s = -glm::sign(q.y);
			vec2 d = min(vec2(dot(a, a), s * (p.x * q.y - p.y * q.x)),
				vec2(glm::dot(b, b), s * (p.y - q.y)));
			return -sqrt(d.x) * glm::sign(d.y);
		}

		real b, h;
	};

	struct LineDivider : public Shape
	{
	private:
		LineDivider();
	public:
		LineDivider(real _m, real _b) {
			m = _m;
			b = _b;
		}
		// above or below the line y = mx + b;

		real Sdf(vec2 p) {
			// we ignore pos for this one
			// positive if above line, negative if below

			vec2 perp = vec2(1, m);
			real y = m * p.x + b;
			
			return p.y - y;
		}

		real m;
		real b;
		//bool above = false;

	};

	struct Intersection : public Shape {
	public:
		Intersection() {

		}

		real Sdf(vec2 p) {
			real sd = -10000.0;
			//std::cout << "size: " << shapes.size() << std::endl;
			for (int i = 0; i < shapes.size(); i++) {
				sd = glm::max(sd, shapes[i].Sdf(p));
			}
			return sd;
		}

		std::vector<LineDivider> shapes;
	};

	//enum BOOLEAN_GEOMETRY_NODE_TYPE {
	//	BOOLEAN_SHAPE,
	//	BOOLEAN_OPERATOR
	//};

	//enum BOOLEAN_GEOMETRY_OPERATOR {
	//	UNION,
	//	INTERSECTION,
	//	SUBTRACTION
	//};

	//struct BooleanGeometryNode {
	//public:
	//	BooleanGeometryNode(std::shared_ptr<Shape> _shape) : m_shape(_shape) { m_nodeType = BOOLEAN_SHAPE; }
	//	BooleanGeometryNode(BOOLEAN_GEOMETRY_OPERATOR _operator) : m_operator(_operator) { m_nodeType = BOOLEAN_OPERATOR; }

	//	
	//	BOOLEAN_GEOMETRY_NODE_TYPE GetNodeType() { return m_nodeType; }
	//	BOOLEAN_GEOMETRY_OPERATOR GetOperator() { return m_operator; }
	//	
	//	std::shared_ptr<BooleanGeometryNode> GetLeft() { return m_left; }
	//	std::shared_ptr<BooleanGeometryNode> GetRight() { return m_right; }

	//	// if shape
	//	std::shared_ptr<Shape> GetShape() { return m_shape; }


	//private:
	//	BOOLEAN_GEOMETRY_NODE_TYPE m_nodeType;
	//	BOOLEAN_GEOMETRY_OPERATOR m_operator = UNION;

	//	std::shared_ptr<BooleanGeometryNode> m_left = nullptr;
	//	std::shared_ptr<BooleanGeometryNode> m_right = nullptr;

	//	// if shape
	//	std::shared_ptr<Shape> m_shape = nullptr;

	//};

	//struct BooleanGeometryTree {
	//	// This is a binary expression tree.
	//	// The leaves MUST be shapes.
	//	// Any parent node MUST have two child nodes.
	//	// Any parent node MUST be an operator.
	//	BooleanGeometryTree() {}

	//	float Sdf(std::shared_ptr<BooleanGeometryNode> node, glm::vec2 p) {

	//		if (node->GetNodeType() == BOOLEAN_SHAPE) {

	//			return node->GetShape()->Sdf(p);
	//		}

	//		else if (node->GetNodeType() == BOOLEAN_OPERATOR) {

	//			switch (node->GetOperator()) {
	//			case UNION:
	//				return fmin(Sdf(node->GetLeft(), p), Sdf(node->GetRight(), p));
	//				break;
	//			case INTERSECTION:
	//				return fmax(Sdf(node->GetLeft(), p), Sdf(node->GetRight(), p));
	//				break;
	//			case SUBTRACTION:
	//				return fmax(Sdf(node->GetLeft(), p), -Sdf(node->GetRight(), p));
	//				break;
	//			default:
	//				return -99.f; // this is an error
	//				break;
	//			}
	//		}

	//		return -108.f; // this is an error;
	//	}


	//	//std::vector<Shape> m_shapeList;
	//	//std::vector<float
	//	std::shared_ptr<BooleanGeometryNode> m_root;

	//};
}