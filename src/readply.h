#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>

#include "vendor/tinyply.cpp"

using namespace std;

struct PLYInfo {
  PLYInfo() : hasColor(false), hasNormals(false), hasFaces(false) {}
  bool hasColor;
  bool hasNormals;
  bool hasFaces;
};

class PLYProperties {
  public:
    vector<string> rgb{"red", "green", "blue"};
    vector<string> position{"x", "y", "z"};
    vector<string> normal{"nx", "ny", "nz", "r", "g", "b"};
    bool hasColor(string property) {
      return find(this->rgb.begin(), this->rgb.end(), property) != this->rgb.end();
    };
    bool hasPosition(string property) {
      return find(this->position.begin(), this->position.end(), property) != this->position.end();
    };
    bool hasNormal(string property) {
      return find(this->normal.begin(), this->normal.end(), property) != this->normal.end();
    };
};

PLYInfo checkPLYProperties(vector<tinyply::PlyElement> elements) {
  PLYInfo info;
  PLYProperties properties;
  for (auto e : elements) {
    for (auto p : e.properties) {
      if (properties.hasColor(p.name) == true) {
        info.hasColor = true;
      }
      if (properties.hasNormal(p.name) == true) {
        info.hasNormals = true;
      }
    }
  }
  return info;
};

class PLYReader {
  public:
    template <typename PointType>
    struct point { 
      PointType x, y, z; 
    };
    template <typename ColorType>
    struct rgb { ColorType r, g, b; };

    template <typename PointType, typename ColorType>
    struct PointData {
      vector<point<PointType>> vertices;
      vector<rgb<ColorType>> color;
    };
    template <typename T>
    static point<float> toFloat(const point<T> _point) {
      point<float> p;
      p.x = (float)_point.x;
      p.y = (float)_point.y;
      p.z = (float)_point.z;
      return p;
    }

    static PointData<float, uint8_t> readply(const string& filepath);
    // can create other functions if needed, i.e. readplydouble
};

PLYReader::PointData<float, uint8_t> PLYReader::readply(const string& filepath) {
  PointData<float, uint8_t> plyData;

  try {
    ifstream filestream(filepath, ios::binary);
    if (filestream.fail()) throw runtime_error("Failed to open " + filepath);
    tinyply::PlyFile file;
    file.parse_header(filestream);
    shared_ptr<PlyData> vertices, col;
    PLYInfo info = checkPLYProperties(file.get_elements());

    // Check for properties ------------------------------------------------------------------
    try { vertices = file.request_properties_from_element("vertex", { "x", "y", "z" }); }
		catch (const exception & e) { cerr << "tinyply exception: " << e.what() << endl; }
    
    if (info.hasColor == true) {
      try { col = file.request_properties_from_element("vertex", { "red", "green", "blue" }); }
  		catch (const exception & e) { cerr << "tinyply exception: " << e.what() << endl; }
    }

    file.read(filestream);

    // Vertices -------------------------------------------------------------------------------

    const size_t verticesBytes = vertices->buffer.size_bytes();
    vector<PLYReader::point<float>> points(vertices->count);
    if (vertices->t == tinyply::Type::FLOAT64) {
      /* Attempting to cast to another type results in munmap_chunk error so we need to read in
         as tinyply::Type and do explicit conversion via transform - will break that into its own function 
      */
      vector<PLYReader::point<double>> tmp(vertices->count);
      memcpy(tmp.data(), vertices->buffer.get(), verticesBytes);
      transform(tmp.begin(), tmp.end(), points.begin(), PLYReader::toFloat<double>);
    } else {
      memcpy(points.data(), vertices->buffer.get(), verticesBytes);
    }
    
    plyData.vertices = points;

    // Color ----------------------------------------------------------------------------------

    if (info.hasColor == true) {
      // TODO check if color is float32
      const size_t rgbBytes = col->buffer.size_bytes();
      vector<rgb<uint8_t>> color(col->count);
      memcpy(color.data(), col->buffer.get(), rgbBytes);
      plyData.color = color;
    }

    return plyData;
  } catch(exception &e) {
    cerr << "Caught tinyply exception: " << e.what() << endl;
    return plyData;
  }
}