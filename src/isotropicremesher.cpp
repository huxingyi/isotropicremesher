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
#include <string>
#include <iostream>
#include "isotropicremesher.h"
#include "halfedgemesh.h"

IsotropicRemesher::~IsotropicRemesher()
{
    delete m_halfedgeMesh;
}

void IsotropicRemesher::remesh(size_t iteration)
{
    delete m_halfedgeMesh;
    m_halfedgeMesh = new HalfedgeMesh(*m_vertices, *m_triangles);
    
    auto targetLength = m_halfedgeMesh->averageEdgeLength();
    
    std::cout << "targetLength:" << targetLength << std::endl;
    
    double minTargetLength = 4.0 / 5.0 * targetLength;
    double maxTargetLength= 4.0 / 3.0 * targetLength;
    
    double minTargetLengthSquared = minTargetLength * minTargetLength;
    double maxTargetLengthSquared = maxTargetLength * maxTargetLength;
    
    for (size_t i = 0; i < iteration; ++i) {
        splitLongEdges(maxTargetLengthSquared);
        collapseShortEdges(minTargetLengthSquared, maxTargetLengthSquared);
        flipEdges();
        shiftVertices();
        projectVertices();
    }
}

void IsotropicRemesher::splitLongEdges(double maxEdgeLengthSquared)
{
    for (HalfedgeMesh::Face *face = m_halfedgeMesh->moveToNextFace(nullptr); 
            nullptr != face; 
            ) {
        //std::cout << "Face:" << face->debugIndex << std::endl;
        const auto &startHalfedge = face->halfedge;
        face = m_halfedgeMesh->moveToNextFace(face);
        HalfedgeMesh::Halfedge *halfedge = startHalfedge;
        do {
            //std::cout << "halfedge:" << (int)halfedge << std::endl;
            const auto &nextHalfedge = halfedge->nextHalfedge;
            double lengthSquared = (halfedge->startVertex->position - nextHalfedge->startVertex->position).lengthSquared();
            if (lengthSquared > maxEdgeLengthSquared) {
                //std::cout << "Break edge at lengthSquared:" << lengthSquared << " maxEdgeLengthSquared:" << maxEdgeLengthSquared << std::endl;
                m_halfedgeMesh->breakEdge(halfedge);
                break;
            }
            halfedge = nextHalfedge;
        } while (halfedge != startHalfedge);
    }
}

HalfedgeMesh *IsotropicRemesher::remeshedHalfedgeMesh()
{
    return m_halfedgeMesh;
}

void IsotropicRemesher::collapseShortEdges(double minEdgeLengthSquared, double maxEdgeLengthSquared)
{
    for (HalfedgeMesh::Face *face = m_halfedgeMesh->moveToNextFace(nullptr); 
            nullptr != face; 
            ) {
        if (face->removed) {
            face = m_halfedgeMesh->moveToNextFace(face);
            continue;
        }
        //std::cout << "Face:" << face->debugIndex << std::endl;
        const auto &startHalfedge = face->halfedge;
        face = m_halfedgeMesh->moveToNextFace(face);
        HalfedgeMesh::Halfedge *halfedge = startHalfedge;
        do {
            //std::cout << "halfedge:" << halfedge->debugIndex << std::endl;
            const auto &nextHalfedge = halfedge->nextHalfedge;
            double lengthSquared = (halfedge->startVertex->position - nextHalfedge->startVertex->position).lengthSquared();
            if (lengthSquared < minEdgeLengthSquared) {
                //std::cout << "Collapse edge at lengthSquared:" << lengthSquared << " minEdgeLengthSquared:" << minEdgeLengthSquared << std::endl;
                if (m_halfedgeMesh->collapseEdge(halfedge, maxEdgeLengthSquared)) {
                    //std::cout << "Collapsed" << std::endl;
                    break;
                }
                //std::cout << "Not collapse" << std::endl;
            }
            halfedge = nextHalfedge;
        } while (halfedge != startHalfedge);
    }
}

void IsotropicRemesher::flipEdges()
{
    // TODO:
}

void IsotropicRemesher::shiftVertices()
{
    // TODO:
}

void IsotropicRemesher::projectVertices()
{
    // TODO:
}

