
#include "Vertex.hpp"

// Конструктор с параметрами (инициализация конфигурации)

MDP::MSIRRT::Vertex::Vertex(const std::vector<double> &coords_, std::pair<int, int> safe_interval_) : coords(coords_.data()), safe_interval(safe_interval_), parent(nullptr) {};
MDP::MSIRRT::Vertex::Vertex(const MDP::MSIRRT::Vertex::VertexCoordType &coords_, std::pair<int, int> safe_interval_) : coords(coords_), safe_interval(safe_interval_), parent(nullptr) {};

// destructor
MDP::MSIRRT::Vertex::~Vertex() {};
