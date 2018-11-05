
#ifndef DIGRAPH_HPP
#define DIGRAPH_HPP

#include <exception>
#include <functional>
#include <list>
#include <map>
#include <utility>
#include <vector>
#include <algorithm>
#include <queue>



// DigraphExceptions are thrown from some of the member functions in the
// Digraph class template, so that exception is declared here, so it
// will be available to any code that includes this header file.

class DigraphException : public std::runtime_error
{
public:
    DigraphException(const std::string& reason);
};


inline DigraphException::DigraphException(const std::string& reason)
    : std::runtime_error{reason}
{
}



// A DigraphEdge lists a "from vertex" (the number of the vertex from which
// the edge points), a "to vertex" (the number of the vertex to which the
// edge points), and an EdgeInfo object.  Because different kinds of Digraphs
// store different kinds of edge information, DigraphEdge is a struct template.

template <typename EdgeInfo>
struct DigraphEdge
{
    int fromVertex;
    int toVertex;
    EdgeInfo einfo;

    //DigraphEdge(int newFromVertex, int newToVertex, EdgeInfo newEinfo);
};



// A DigraphVertex includes two things: a VertexInfo object and a list of
// its outgoing edges.  Because different kinds of Digraphs store different
// kinds of vertex and edge information, DigraphVertex is a struct template.

template <typename VertexInfo, typename EdgeInfo>
struct DigraphVertex
{
    VertexInfo vinfo;
    std::list<DigraphEdge<EdgeInfo>> edges;

    //DigraphVertex(VertexInfo newVinfo, std::list<DigraphEdge<EdgeInfo>> newEdges);
};



// Digraph is a class template that represents a directed graph implemented
// using adjacency lists.  It takes two type parameters:
//
// * VertexInfo, which specifies the kind of object stored for each vertex
// * EdgeInfo, which specifies the kind of object stored for each edge
//
// You'll need to implement the member functions declared here; each has a
// comment detailing how it is intended to work.
//
// Each vertex in a Digraph is identified uniquely by a "vertex number".
// Vertex numbers are not necessarily sequential and they are not necessarily
// zero- or one-based.

template <typename VertexInfo, typename EdgeInfo>
class Digraph
{
public:
    // The default constructor initializes a new, empty Digraph so that
    // contains no vertices and no edges.
    Digraph();

    // The copy constructor initializes a new Digraph to be a deep copy
    // of another one (i.e., any change to the copy will not affect the
    // original).
    Digraph(const Digraph& d);

    // The move constructor initializes a new Digraph from an expiring one.
    Digraph(Digraph&& d) noexcept;

    // The destructor deallocates any memory associated with the Digraph.
    ~Digraph() noexcept;

    // The assignment operator assigns the contents of the given Digraph
    // into "this" Digraph, with "this" Digraph becoming a separate, deep
    // copy of the contents of the given one (i.e., any change made to
    // "this" Digraph afterward will not affect the other).
    Digraph& operator=(const Digraph& d);

    // The move assignment operator assigns the contents of an expiring
    // Digraph into "this" Digraph.
    Digraph& operator=(Digraph&& d) noexcept;

    // vertices() returns a std::vector containing the vertex numbers of
    // every vertex in this Digraph.
    std::vector<int> vertices() const;

    // edges() returns a std::vector of std::pairs, in which each pair
    // contains the "from" and "to" vertex numbers of an edge in this
    // Digraph.  All edges are included in the std::vector.
    std::vector<std::pair<int, int>> edges() const;

    // This overload of edges() returns a std::vector of std::pairs, in
    // which each pair contains the "from" and "to" vertex numbers of an
    // edge in this Digraph.  Only edges outgoing from the given vertex
    // number are included in the std::vector.  If the given vertex does
    // not exist, a DigraphException is thrown instead.
    std::vector<std::pair<int, int>> edges(int vertex) const;

    // vertexInfo() returns the VertexInfo object belonging to the vertex
    // with the given vertex number.  If that vertex does not exist, a
    // DigraphException is thrown instead.
    VertexInfo vertexInfo(int vertex) const;

    // edgeInfo() returns the EdgeInfo object belonging to the edge
    // with the given "from" and "to" vertex numbers.  If either of those
    // vertices does not exist *or* if the edge does not exist, a
    // DigraphException is thrown instead.
    EdgeInfo edgeInfo(int fromVertex, int toVertex) const;

    // addVertex() adds a vertex to the Digraph with the given vertex
    // number and VertexInfo object.  If there is already a vertex in
    // the graph with the given vertex number, a DigraphException is
    // thrown instead.
    void addVertex(int vertex, const VertexInfo& vinfo);

    // addEdge() adds an edge to the Digraph pointing from the given
    // "from" vertex number to the given "to" vertex number, and
    // associates with the given EdgeInfo object with it.  If one
    // of the vertices does not exist *or* if the same edge is already
    // present in the graph, a DigraphException is thrown instead.
    void addEdge(int fromVertex, int toVertex, const EdgeInfo& einfo);

    // removeVertex() removes the vertex (and all of its incoming
    // and outgoing edges) with the given vertex number from the
    // Digraph.  If the vertex does not exist already, a DigraphException
    // is thrown instead.
    void removeVertex(int vertex);

    // removeEdge() removes the edge pointing from the given "from"
    // vertex number to the given "to" vertex number from the Digraph.
    // If either of these vertices does not exist *or* if the edge
    // is not already present in the graph, a DigraphException is
    // thrown instead.
    void removeEdge(int fromVertex, int toVertex);

    // vertexCount() returns the number of vertices in the graph.
    int vertexCount() const noexcept;

    // edgeCount() returns the total number of edges in the graph,
    // counting edges outgoing from all vertices.
    int edgeCount() const noexcept;

    // This overload of edgeCount() returns the number of edges in
    // the graph that are outgoing from the given vertex number.
    // If the given vertex does not exist, a DigraphException is
    // thrown instead.
    int edgeCount(int vertex) const;

    // isStronglyConnected() returns true if the Digraph is strongly
    // connected (i.e., every vertex is reachable from every other),
    // false otherwise.
    bool isStronglyConnected() const;

    // findShortestPaths() takes a start vertex number and a function
    // that takes an EdgeInfo object and determines an edge weight.
    // It uses Dijkstra's Shortest Path Algorithm to determine the
    // shortest paths from the start vertex to every other vertex
    // in the graph.  The result is returned as a std::map<int, int>
    // where the keys are vertex numbers and the value associated
    // with each key k is the precedessor of that vertex chosen by
    // the algorithm.  For any vertex without a predecessor (e.g.,
    // a vertex that was never reached, or the start vertex itself),
    // the value is simply a copy of the key.
    std::map<int, int> findShortestPaths(
        int startVertex,
        std::function<double(const EdgeInfo&)> edgeWeightFunc) const;


private:
    // Add whatever member variables you think you need here.  One
    // possibility is a std::map where the keys are vertex numbers
    // and the values are DigraphVertex<VertexInfo, EdgeInfo> objects.
    std::map<int, DigraphVertex<VertexInfo,EdgeInfo>> graph;

    // You can also feel free to add any additional member functions
    // you'd like (public or private), so long as you don't remove or
    // change the signatures of the ones that already exist.

    //void printGraph(const std::map<int,DigraphVertex<VertexInfo,EdgeInfo>>& g);
    std::vector<int> DFTr(const std::map<int,DigraphVertex<VertexInfo,EdgeInfo>>& g, 
        int v, std::vector<int> visited) const;
};



// You'll need to implement the member functions below.  There's enough
// code in place to make them compile, but they'll all need to do the
// correct thing instead.

/*
template <typename VertexInfo, typename EdgeInfo>
DigraphVertex<VertexInfo, EdgeInfo>::DigraphVertex(VertexInfo newVinfo, 
    std::list<DigraphEdge<EdgeInfo>> newEdges)
    : vinfo(newVinfo), edges(newEdges)
{
}

template <typename EdgeInfo>
DigraphEdge<EdgeInfo>::DigraphEdge(int newFromVertex, int newToVertex, EdgeInfo newEinfo)
    : fromVertex(newFromVertex), toVertex(newToVertex), einfo(newEinfo)
{
}
*/

template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::Digraph()
{
}


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::Digraph(const Digraph& d)
{
    graph = d.graph;
}


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::Digraph(Digraph&& d) noexcept
{
    std::swap(graph, d.graph);
}


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::~Digraph() noexcept
{
}


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>& Digraph<VertexInfo, EdgeInfo>::operator=(const Digraph& d)
{
    graph = d.graph;
    return *this;
}


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>& Digraph<VertexInfo, EdgeInfo>::operator=(Digraph&& d) noexcept
{
    std::swap(graph, d.graph);
    return *this;
}


template <typename VertexInfo, typename EdgeInfo>
std::vector<int> Digraph<VertexInfo, EdgeInfo>::vertices() const
{
    typename std::map<int,DigraphVertex<VertexInfo,EdgeInfo>>::const_iterator it;
    std::vector<int> v;

    for (it = graph.begin(); it != graph.end(); ++it)
    {   
        v.push_back(it->first);
    }
    return v;
}


template <typename VertexInfo, typename EdgeInfo>
std::vector<std::pair<int, int>> Digraph<VertexInfo, EdgeInfo>::edges() const
{
    std::vector<std::pair<int,int>> e;

    typename std::map<int,DigraphVertex<VertexInfo,EdgeInfo>>::const_iterator it;
    typename std::list<DigraphEdge<EdgeInfo>>::const_iterator itlist;

    for (it = graph.begin(); it != graph.end(); ++it)
    {
        for (itlist = it->second.edges.begin(); itlist != it->second.edges.end(); ++itlist)
        {
            std::pair edge(itlist->fromVertex, itlist->toVertex);
            e.push_back(edge);
        }
    }

    return e;
}


template <typename VertexInfo, typename EdgeInfo>
std::vector<std::pair<int, int>> Digraph<VertexInfo, EdgeInfo>::edges(int vertex) const
{
    std::vector<std::pair<int,int>> edgesVector;
    if (graph.find(vertex) == graph.end())
    {
        throw DigraphException("VERTEX DOES NOT EXIST");
    }
    else
    {
        typename std::list<DigraphEdge<EdgeInfo>>::const_iterator itlist;
        
        for (itlist = graph.at(vertex).edges.begin(); itlist != graph.at(vertex).edges.end(); ++itlist)
        {
            std::pair edge(itlist->fromVertex, itlist->toVertex);
            edgesVector.push_back(edge);
        }             
    }
    return edgesVector;
}


template <typename VertexInfo, typename EdgeInfo>
VertexInfo Digraph<VertexInfo, EdgeInfo>::vertexInfo(int vertex) const
{
    if (graph.find(vertex) == graph.end())
    {
        throw DigraphException("VERTEX DOES NOT EXIST");
    }
    else
    {
        return graph.at(vertex).vinfo;
    }
}


template <typename VertexInfo, typename EdgeInfo>
EdgeInfo Digraph<VertexInfo, EdgeInfo>::edgeInfo(int fromVertex, int toVertex) const
{
    typename std::map<int,DigraphVertex<VertexInfo,EdgeInfo>>::const_iterator it;
    typename std::list<DigraphEdge<EdgeInfo>>::const_iterator itlist;

    bool edgeFound = false;
    EdgeInfo edge;

    for (it = graph.begin(); it != graph.end(); ++it)
    {
        for (itlist = it->second.edges.begin(); itlist != it->second.edges.end(); ++itlist)
        {
            if (itlist->fromVertex == fromVertex && itlist->toVertex == toVertex)
            {
                edge = itlist->einfo;
                edgeFound = true;
                return edge;
            }           
        }   
    }
    if (edgeFound == false)
    {
        throw DigraphException("INVALID EDGE");
    }
}


template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::addVertex(int vertex, const VertexInfo& vinfo)
{
    typename std::map<int,DigraphVertex<VertexInfo,EdgeInfo>>::const_iterator it;

    if (graph.find(vertex) != graph.end())
    {
        throw DigraphException("VERTEX EXISTS");
    }
    for (it = graph.begin(); it != graph.end(); ++it)
    {
        if (it->second.vinfo == vinfo)
        {
            throw DigraphException("VERTEX EXISTS");
        }
    }
    DigraphVertex<VertexInfo, EdgeInfo> newVertex;
    newVertex.vinfo = vinfo;

    graph.insert(std::pair<int,DigraphVertex<VertexInfo,EdgeInfo>>(vertex,newVertex));
    //printGraph(graph);
}


template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::addEdge(int fromVertex, int toVertex, const EdgeInfo& einfo)
{
    if (graph.find(fromVertex) == graph.end() || graph.find(toVertex) == graph.end())
    {
        throw DigraphException("VERTEX DOES NOT EXIST");
    }

    typename std::list<DigraphEdge<EdgeInfo>>::const_iterator itlist;

    for (itlist = graph.at(fromVertex).edges.begin(); 
        itlist != graph.at(fromVertex).edges.end(); ++itlist)
    {
        if (itlist->fromVertex == fromVertex && itlist->toVertex == toVertex)
        {
            throw DigraphException("EDGE EXIST ERROR");
        }
    }

    DigraphEdge<EdgeInfo> newEdge{fromVertex, toVertex, einfo};
    graph.at(fromVertex).edges.push_back(newEdge);

    //printGraph(graph);
}


template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::removeVertex(int vertex)
{
    typename std::map<int,DigraphVertex<VertexInfo,EdgeInfo>>::const_iterator it;

    it = graph.find(vertex);

    if (it != graph.end())
    {
        graph.erase(it);
    }
    else
    {
        throw DigraphException("VERTEX DOES NOT EXIST ERROR");
    }
}


template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::removeEdge(int fromVertex, int toVertex)
{
    if (graph.find(fromVertex) == graph.end() || graph.find(toVertex) == graph.end())
    {
        throw DigraphException("VERTEX DOES NOT EXIST ERROR");
    }

    typename std::map<int,DigraphVertex<VertexInfo,EdgeInfo>>::const_iterator it;
    typename std::list<DigraphEdge<EdgeInfo>>::const_iterator itlist;
    bool erased = false;

    for (it = graph.begin(); it != graph.end(); ++it)
    {
        for (itlist = it->second.edges.begin(); itlist != it->second.edges.end(); ++itlist)
        {
            if (itlist->fromVertex == fromVertex && itlist->toVertex == toVertex)
            {
                graph.at(fromVertex).edges.erase(itlist);
                erased = true;
            }
        }
    }
    if (!erased)
    {
        throw DigraphException("EDGE DOES NOT EXIST ERROR");
    }
}


template <typename VertexInfo, typename EdgeInfo>
int Digraph<VertexInfo, EdgeInfo>::vertexCount() const noexcept
{
    return graph.size();
}


template <typename VertexInfo, typename EdgeInfo>
int Digraph<VertexInfo, EdgeInfo>::edgeCount() const noexcept
{
    int edgeSize = 0;

    typename std::map<int,DigraphVertex<VertexInfo,EdgeInfo>>::const_iterator it;

    for (it = graph.begin(); it != graph.end(); ++it)
    {
        edgeSize += graph.at(it->first).edges.size();
    }
    

    return edgeSize;
}


template <typename VertexInfo, typename EdgeInfo>
int Digraph<VertexInfo, EdgeInfo>::edgeCount(int vertex) const
{
    return graph.at(vertex).edges.size();
}


template <typename VertexInfo, typename EdgeInfo>
bool Digraph<VertexInfo, EdgeInfo>::isStronglyConnected() const
{
    typename std::map<int,DigraphVertex<VertexInfo,EdgeInfo>>::const_iterator it;

    std::vector<int> existing = vertices();

    for (it = graph.begin(); it != graph.end(); ++it)
    {
        std::vector<int> visited;

        if (std::find(visited.begin(), visited.end(), it->first) == visited.end())
        {
            visited = DFTr(graph, it->first, visited);

            if (visited.size() < existing.size() - 1)
            {
                return false;
            }
        }
    }
    return true;
}


template <typename VertexInfo, typename EdgeInfo>
std::map<int, int> Digraph<VertexInfo, EdgeInfo>::findShortestPaths(
    int startVertex,
    std::function<double(const EdgeInfo&)> edgeWeightFunc) const
{
    typename std::map<int,DigraphVertex<VertexInfo,EdgeInfo>>::const_iterator it;
    typename std::list<DigraphEdge<EdgeInfo>>::const_iterator itlist;

    std::map<int,double> distance;
    std::map<int,int> predecessor; 
    std::vector<int> unvisited;

    for (it = graph.begin(); it !=  graph.end(); ++it)
    {
        unvisited.push_back(it->first);
        distance[it->first] = std::numeric_limits<double>::infinity();
    }

    distance[startVertex] = 0;
    predecessor[startVertex] = startVertex;

    auto comp = [distance](const int& lhs, const int& rhs){
        return distance.at(lhs) < distance.at(rhs);};

    std::priority_queue<int, std::vector<int>, decltype(comp)> pq(comp);

    pq.push(startVertex);

    while (!pq.empty())
    {
        int v = pq.top();
        pq.pop();

        std::vector<int>::const_iterator it;
        it = std::find(unvisited.begin(), unvisited.end(), v);

        if (it != unvisited.end())
        {
            unvisited.erase(it);

            for (itlist = graph.at(v).edges.begin(); 
                itlist != graph.at(v).edges.end(); ++itlist)
            {
                double d = distance.at(v) + edgeWeightFunc(itlist->einfo);
                if (distance.at(itlist->toVertex) > d)
                {
                    distance[itlist->toVertex] = d;
                    predecessor[itlist->toVertex] = v;
                    pq.push(itlist->toVertex);

                }   
            }
        }
    }
    return predecessor;
}

/*
template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo,EdgeInfo>::printGraph(const std::map<int,DigraphVertex<VertexInfo,EdgeInfo>>& g)
{
    typename std::map<int,DigraphVertex<VertexInfo,EdgeInfo>>::const_iterator it;
    typename std::list<DigraphEdge<EdgeInfo>>::const_iterator itlist;


    for (it = g.begin(); it != g.end(); ++it)
    {
        std::cout << it->first << ": ";


        for (itlist = it->second.edges.begin(); itlist != it->second.edges.end(); ++itlist)
        {
            std::cout << "(" << itlist->fromVertex << ", " << itlist->toVertex << ", " << itlist->einfo << ")";
            std::cout << " -> ";
        }
        std::cout << std::endl;
    }

}
*/

template <typename VertexInfo, typename EdgeInfo>
std::vector<int> Digraph<VertexInfo,EdgeInfo>::DFTr(const std::map<int,DigraphVertex<VertexInfo,EdgeInfo>>& g, 
    int v, std::vector<int> visited) const
{
    typename std::list<DigraphEdge<EdgeInfo>>::const_iterator itlist;

    std::vector<int> existing = vertices();

    for (itlist = graph.at(v).edges.begin(); 
        itlist != graph.at(v).edges.end(); ++itlist)
    {
        if (std::find(existing.begin(), existing.end(), itlist->toVertex) != existing.end()
            && std::find(visited.begin(), visited.end(), itlist->toVertex) == visited.end())
        {
            visited.push_back(itlist->toVertex);
            visited = DFTr(graph, itlist->toVertex, visited);
        }
    }
    return visited;
}

#endif // DIGRAPH_HPP

