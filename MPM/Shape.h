#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <vector>
#include <iostream>
#include <math.h>

namespace sdf {
	struct Shape {
	public:

		virtual float Sdf(glm::vec2 p) = 0;

		float SdfRounded(glm::vec2 p, float r) {
			return Sdf(p) - r;
		}

		float SdfHollow(glm::vec2 p, float r1, float r2) {
			// r1 is inner rounding, r2 is outer rounding
			// r1 = 0 and r2 = 0 => the point cloud is empty
			// r1 can at most be the min distance to the center-line/skeleton of the shape
			return glm::max(-SdfRounded(p, -r1), SdfRounded(p, r2));
		}

		glm::vec2 pos;
	};

	struct Circle : public Shape {
	private:
		Circle() {}

	public:
		Circle(glm::vec2 _pos, float _r) {
			pos = _pos;
			r = _r;
		}

		float Sdf(glm::vec2 p) {
			return glm::length(p - pos) - r;
		}

		float r;
	};

	//struct Donut : public Shape {
	//private:
	//	Donut() {}
	//public:
	//	Donut(glm::vec2 _pos, float _r1, float _r2) {
	//		pos = _pos;
	//		r1 = _r1;
	//		r2 = _r2;
	//	}

	//	float Sdf(glm::vec2 p) {
	//		float d = glm::length(p - pos);
	//		return glm::max(d - r2, r1 - d);
	//	}

	//	float r1; // inner radius
	//	float r2; // outer radius
	//};

	struct Rectangle : public Shape {
	private:
		Rectangle() {}
	public:
		Rectangle(glm::vec2 _pos, float _b, float _h) {
			pos = _pos;
			b = _b;
			h = _h;
		}

		float Sdf(glm::vec2 p) {
			glm::vec2 d = glm::abs(p - pos) - glm::vec2(b, h);

			return glm::length(glm::max(d, glm::vec2(0.f))) + glm::min(glm::max(d.x, d.y), 0.0f);
		}

		float b, h; // b = horizontal base length, h = vertical height length;
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