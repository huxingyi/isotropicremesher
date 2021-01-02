/*
 *  Copyright (c) 2020-2021 Jeremy HU <jeremy-at-dust3d dot org>. All rights reserved. 
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
#ifndef ISOTROPICREMESHER_H
#define ISOTROPICREMESHER_H
#include <vector>
#include "vector3.h"
#include "axisalignedboundingboxtree.h"
#include "axisalignedboundingbox.h"

class HalfedgeMesh;

class IsotropicRemesher
{
public:
    IsotropicRemesher(const std::vector<Vector3> *vertices,
            const std::vector<std::vector<size_t>> *triangles) :
        m_vertices(vertices),
        m_triangles(triangles)
    {
    }
    ~IsotropicRemesher();
    void remesh(size_t iteration);
    HalfedgeMesh *remeshedHalfedgeMesh();
    
private:
    const std::vector<Vector3> *m_vertices = nullptr;
    const std::vector<std::vector<size_t>> *m_triangles = nullptr;
    std::vector<Vector3> *m_triangleNormals = nullptr;
    HalfedgeMesh *m_halfedgeMesh = nullptr;
    std::vector<AxisAlignedBoudingBox> *m_triangleBoxes = nullptr;
    AxisAlignedBoudingBoxTree *m_axisAlignedBoundingBoxTree = nullptr;
    
    void addTriagleToAxisAlignedBoundingBox(const std::vector<size_t> &triangle, AxisAlignedBoudingBox *box)
    {
        for (size_t i = 0; i < 3; ++i)
            box->update((*m_vertices)[triangle[i]]);
    }
    
    void splitLongEdges(double maxEdgeLength);
    void collapseShortEdges(double minEdgeLengthSquared, double maxEdgeLengthSquared);
    void flipEdges();
    void shiftVertices();
    void projectVertices();
    void buildAxisAlignedBoundingBoxTree();
};

#endif
