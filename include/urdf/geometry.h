#ifndef URDF_GEOMETRY_H
#define URDF_GEOMETRY_H

#include <tinyxml/tinyxml.h>

#include "urdf/common.h"
#include "urdf/exception.h"

namespace urdf {

	enum GeometryType {
		SPHERE,
		BOX,
		CYLINDER,
		MESH
	};

	class Geometry {
		public:
			GeometryType type;
			virtual ~Geometry(void) {};

			Geometry(GeometryType type): type(type) {}

			static Geometry* fromXml(TiXmlElement* xml);
	};

	class Sphere : public Geometry {
		public:
			double radius;

			void clear() {
				radius = 0;
			}

			Sphere() : radius(0.), Geometry(GeometryType::SPHERE) {}

			static Sphere* fromXml(TiXmlElement* xml);
	};

	class Box : public Geometry {
		public:
			Vector3 dim;

			void clear() {
				this->dim.clear();
			}

			Box() : Geometry(GeometryType::BOX) {}

			static Box* fromXml(TiXmlElement* xml);
	};

	class Cylinder : public Geometry {
		public:
			double length;
			double radius;

			void clear() {
				length = 0;
				radius = 0;
			}

			Cylinder() : length(0.), radius(0.), Geometry(GeometryType::CYLINDER) {}

			static Cylinder* fromXml(TiXmlElement* xml);
	};

	class Mesh : public Geometry {
		public:
			std::string filename;
			Vector3 scale;

			void clear() {
				filename.clear();

				scale.x = 1;
				scale.y = 1;
				scale.z = 1;
			}

			Mesh() : scale(Vector3(1., 1., 1.)), Geometry(GeometryType::MESH) {}

			static Mesh* fromXml(TiXmlElement* xml);
	};
}

#endif
