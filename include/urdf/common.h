#ifndef URDF_COMMON_H
#define URDF_COMMON_H

#include <vector>
#include <string>
#include <math.h>
#include <algorithm>
#include <cctype>
#include <locale>

#ifndef M_PI
#define M_PI 3.141592538
#endif //M_PI

#include "tinyxml/txml.h"
#include <memory>

#include "urdf/exception.h"

using namespace std;

namespace urdf {

	struct Vector3 {
		double x;
		double y;
		double z;

		void clear() {
			x = 0.;
			y = 0.;
			z = 0.;
		}

		Vector3 operator+(const Vector3& other);

		Vector3(double x, double y, double z) : x(x), y(y), z(z) {}
		Vector3(const Vector3 &other) : x(other.x), y(other.y), z(other.z) {}
		Vector3() : x(0.), y(0.), z(0.) {}

		static Vector3 fromVecStr(const string& vector_str);
	};

	struct Rotation {
		double x;
		double y;
		double z;
		double w;

		void clear() {
			x=0.;
			y=0.;
			z=0.;
			w=1.;
		}

		void getRpy(double &roll, double &pitch, double &yaw) const;
		void normalize();
		Rotation getInverse() const;

		Rotation operator*( const Rotation &other ) const;
		Vector3 operator*(const Vector3& vec) const;

		Rotation(double x, double y, double z, double w) : x(x), y(y), z(z), w(w) {}
		Rotation(const Rotation &other) : x(other.x), y(other.y), z(other.z), w(other.w) {}
		Rotation() : x(0.), y(0.), z(0.), w(1.) {}

		static Rotation fromRpy(double roll, double pitch, double yaw);
		static Rotation fromRpyStr(const string &rotation_str);
	};

	struct Color {
		float r;
		float g;
		float b;
		float a;

		void clear() {
			r = 0.;
			g = 0.;
			b = 0.;
			a = 1.;
		}

		Color() : r(0.), g(0.), b(0.), a(1.) {}
		Color(float r, float g, float b, float a) : r(r), g(g), b(b), a(a) {}
		Color(const Color& other) : r(other.r), g(other.g), b(other.b), a(other.a) {}

		static Color fromColorStr(const std::string &vector_str);
	};

	struct Transform {
		Vector3  position;
		Rotation rotation;

		void clear() {
			this->position.clear();
			this->rotation.clear();
		};

		Transform() : position(Vector3()), rotation(Rotation()) {}
		Transform(const Transform& other) : position(other.position), rotation(other.rotation) {}

		static Transform fromXml(TiXmlElement* xml);
	};


	struct Twist {
		Vector3  linear;
		Vector3  angular;

		void clear() {
			this->linear.clear();
			this->angular.clear();
		}

		Twist() : linear(Vector3()), angular(Vector3()) {}
		Twist(const Twist& other) : linear(other.linear), angular(other.angular) {}
	};
}

namespace nonboost {


inline void split(std::vector<std::string>& tokens,
           std::string s,
           std::string delimiter)
{
    size_t pos = 0;
    while ((pos = s.find(delimiter)) != std::string::npos) {
        tokens.push_back(s.substr(0, pos));
        s.erase(0, pos + delimiter.length());
    }
    tokens.push_back(s);
}

struct bad_lexical_cast : std::invalid_argument
{
    using std::invalid_argument::invalid_argument;
};

template <typename T>
inline T lexical_cast(std::string);

template <>
inline double lexical_cast<double>(std::string str) try
{
    double ret = std::stod(str);
    return ret;
}
catch(std::exception& e)
{
    throw bad_lexical_cast(e.what());
}

namespace algorithm
{
    // trim from start (in place)
    inline void ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
            return !std::isspace(ch);
        }));
    }

    // trim from end (in place)
    inline void rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
            return !std::isspace(ch);
        }).base(), s.end());
    }

    // trim from both ends (in place)
    inline void trim(std::string &s) {
        rtrim(s);
        ltrim(s);
    }
}

inline std::string is_any_of(std::string s)
{
    return s;
}


}

#endif
