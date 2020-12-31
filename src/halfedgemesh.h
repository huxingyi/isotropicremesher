/*
 *  Copyright (c) 2020 Jeremy HU <jeremy-at-dust3d dot org>. All rights reserved. 
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:

 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.

 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */
#ifndef HALFEDGE_MESH_H
#define HALFEDGE_MESH_H
#include "vector3.h"

class HalfedgeMesh
{
public:
    struct Halfedge;
      
    struct Vertex
    {
        Vector3 position;
        Halfedge *firstHalfedge = nullptr;
        Vertex *previousVertex = nullptr;
        Vertex *nextVertex = nullptr;
    };

    struct Face
    {
        Halfedge *halfedge = nullptr;
        Face *previousFace = nullptr;
        Face *nextFace = nullptr;
    };

    struct Halfedge
    {
        Vertex *startVertex = nullptr;
        Face *leftFace = nullptr;
        Halfedge *nextHalfedge = nullptr;
        Halfedge *previousHalfedge = nullptr;
        Halfedge *oppositeHalfedge = nullptr;
    };
    
    HalfedgeMesh(const std::vector<Vector3> &vertices,
        const std::vector<std::vector<size_t>> &faces);
    ~HalfedgeMesh();
    
    double averageEdgeLength();
    
private:
    Vertex *m_firstVertex = nullptr;
    Vertex *m_lastVertex = nullptr;
    Face *m_firstFace = nullptr;
    Face *m_lastFace = nullptr;
    
    static inline uint64_t makeHalfedgeKey(size_t first, size_t second)
    {
        return (first << 32) | second;
    }
    
    static inline uint64_t swapHalfedgeKey(uint64_t key)
    {
        return makeHalfedgeKey(key >> 32, key & 0xffffffff);
    }
};

#endif
