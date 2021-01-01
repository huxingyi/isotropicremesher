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
#include <unordered_map>
#include "halfedgemesh.h"

HalfedgeMesh::~HalfedgeMesh()
{
    // TODO:
}

HalfedgeMesh::HalfedgeMesh(const std::vector<Vector3> &vertices,
    const std::vector<std::vector<size_t>> &faces)
{
    std::vector<Vertex *> halfedgeVertices;
    halfedgeVertices.reserve(vertices.size());
    for (const auto &it: vertices) {
        Vertex *vertex = new Vertex;
        vertex->position = it;
        halfedgeVertices.push_back(vertex);
    }
    
    for (size_t vertexIndex = 1; vertexIndex < halfedgeVertices.size(); ++vertexIndex) {
        auto &previous = halfedgeVertices[vertexIndex - 1];
        auto &current = halfedgeVertices[vertexIndex];
        current->previousVertex = previous;
        previous->nextVertex = current;
    }
    
    if (!halfedgeVertices.empty()) {
        m_firstVertex = halfedgeVertices.front();
        m_lastVertex = halfedgeVertices.back();
    }
    
    std::vector<Face *> halfedgeFaces;
    halfedgeFaces.reserve(faces.size());
    for (const auto &it: faces) {
        Face *face = new Face;
        face->debugIndex = ++m_debugFaceIndex;
        halfedgeFaces.push_back(face);
    }
    
    if (!halfedgeFaces.empty()) {
        m_firstFace = halfedgeFaces.front();
        m_lastFace = halfedgeFaces.back();
    }
    
    for (size_t faceIndex = 1; faceIndex < halfedgeFaces.size(); ++faceIndex) {
        auto &previous = halfedgeFaces[faceIndex - 1];
        auto &current = halfedgeFaces[faceIndex];
        current->previousFace = previous;
        previous->nextFace = current;
    }

    std::unordered_map<uint64_t, Halfedge *> halfedgeMap;
    for (size_t faceIndex = 0; faceIndex < faces.size(); ++faceIndex) {
        const auto &indices = faces[faceIndex];
        if (3 != indices.size()) {
            std::cerr << "Found non-triangle, face count:" << indices.size() << std::endl;
            continue;
        }
        std::vector<Halfedge *> halfedges(indices.size());
        for (size_t i = 0; i < 3; ++i) {
            size_t j = (i + 1) % 3;
            const auto &first = indices[i];
            const auto &second = indices[j];
            
            Vertex *vertex = halfedgeVertices[first];
            Halfedge *halfedge = new Halfedge;
            halfedge->startVertex = vertex;
            halfedge->leftFace = halfedgeFaces[faceIndex];
            
            if (nullptr == halfedge->leftFace->halfedge) {
                halfedge->leftFace->halfedge = halfedge;
                if (nullptr == vertex->firstHalfedge) {
                    vertex->firstHalfedge = halfedge;
                }
            }
            
            halfedges[i] = halfedge;
            
            auto insertResult = halfedgeMap.insert({makeHalfedgeKey(first, second), halfedge});
            if (!insertResult.second) {
                std::cerr << "Found repeated halfedge:" << first << "," << second << std::endl;
            }
        }
        linkFaceHalfedges(halfedges);
    }
    for (auto &it: halfedgeMap) {
        auto halfedgeIt = halfedgeMap.find(swapHalfedgeKey(it.first));
        if (halfedgeIt == halfedgeMap.end())
            continue;
        it.second->oppositeHalfedge = halfedgeIt->second;
        halfedgeIt->second->oppositeHalfedge = it.second;
    }
}

void HalfedgeMesh::linkFaceHalfedges(std::vector<HalfedgeMesh::Halfedge *> &halfedges)
{
    for (size_t i = 0; i < halfedges.size(); ++i) {
        size_t j = (i + 1) % halfedges.size();
        halfedges[i]->nextHalfedge = halfedges[j];
        halfedges[j]->previousHalfedge = halfedges[i];
    }
}

void HalfedgeMesh::updateFaceHalfedgesLeftFace(std::vector<HalfedgeMesh::Halfedge *> &halfedges,
    HalfedgeMesh::Face *leftFace)
{
    for (auto &it: halfedges)
        it->leftFace = leftFace;
}

void HalfedgeMesh::linkHalfedgePair(HalfedgeMesh::Halfedge *first, HalfedgeMesh::Halfedge *second)
{
    first->oppositeHalfedge = second;
    second->oppositeHalfedge = first;
}

double HalfedgeMesh::averageEdgeLength()
{
    double totalLength = 0.0;
    size_t halfedgeCount = 0;
    for (Face *face = m_firstFace; nullptr != face; face = face->nextFace) {
        const auto &startHalfedge = face->halfedge;
        Halfedge *halfedge = startHalfedge;
        do {
            const auto &nextHalfedge = halfedge->nextHalfedge;
            totalLength += (halfedge->startVertex->position - nextHalfedge->startVertex->position).length();
            ++halfedgeCount;
        } while (halfedge != startHalfedge);
    }
    if (0 == halfedgeCount)
        return 0.0;
    return totalLength / halfedgeCount;
}

HalfedgeMesh::Face *HalfedgeMesh::newFace()
{
    Face *face = new Face;
    face->debugIndex = ++m_debugFaceIndex;
    if (nullptr != m_lastFace) {
        m_lastFace->nextFace = face;
        face->previousFace = m_lastFace;
    } else {
        m_firstFace = face;
    }
    m_lastFace = face;
    return face;
}

HalfedgeMesh::Vertex *HalfedgeMesh::newVertex()
{
    Vertex *vertex = new Vertex;
    if (nullptr != m_lastVertex) {
        m_lastVertex->nextVertex = vertex;
        vertex->previousVertex = m_lastVertex;
    } else {
        m_firstVertex = vertex;
    }
    m_lastVertex = vertex;
    return vertex;
}

HalfedgeMesh::Face *HalfedgeMesh::moveToNextFace(HalfedgeMesh::Face *face)
{
    if (nullptr == face) {
        face = m_firstFace;
        if (nullptr == face)
            return nullptr;
        if (!face->removed)
            return face;
    }
    do {
        face = face->nextFace;
    } while (nullptr != face && face->removed);
    return face;
}

HalfedgeMesh::Vertex *HalfedgeMesh::moveToNextVertex(HalfedgeMesh::Vertex *vertex)
{
    if (nullptr == vertex) {
        vertex = m_firstVertex;
        if (nullptr == vertex)
            return nullptr;
        if (!vertex->removed)
            return vertex;
    }
    do {
        vertex = vertex->nextVertex;
    } while (nullptr != vertex && vertex->removed);
    return vertex;
}

void HalfedgeMesh::breakFace(HalfedgeMesh::Face *leftOldFace,
    HalfedgeMesh::Halfedge *halfedge,
    HalfedgeMesh::Vertex *breakPointVertex,
    std::vector<HalfedgeMesh::Halfedge *> &leftNewFaceHalfedges,
    std::vector<HalfedgeMesh::Halfedge *> &leftOldFaceHalfedges)
{
    std::vector<Halfedge *> leftFaceHalfedges = {
        halfedge->previousHalfedge,
        halfedge,
        halfedge->nextHalfedge
    };
    
    Face *leftNewFace = newFace();
    leftNewFace->halfedge = leftFaceHalfedges[2];
    leftFaceHalfedges[2]->leftFace = leftNewFace;
    
    leftNewFaceHalfedges.push_back(new Halfedge);
    leftNewFaceHalfedges.push_back(new Halfedge);
    leftNewFaceHalfedges.push_back(leftFaceHalfedges[2]);
    linkFaceHalfedges(leftNewFaceHalfedges);
    updateFaceHalfedgesLeftFace(leftNewFaceHalfedges, leftNewFace);
    leftNewFaceHalfedges[0]->startVertex = leftFaceHalfedges[0]->startVertex;
    leftNewFaceHalfedges[1]->startVertex = breakPointVertex;
    
    leftOldFaceHalfedges.push_back(leftFaceHalfedges[0]);
    leftOldFaceHalfedges.push_back(halfedge);
    leftOldFaceHalfedges.push_back(new Halfedge);
    linkFaceHalfedges(leftOldFaceHalfedges);
    updateFaceHalfedgesLeftFace(leftOldFaceHalfedges, leftOldFace);
    leftOldFaceHalfedges[2]->startVertex = breakPointVertex;
    
    breakPointVertex->firstHalfedge = leftNewFaceHalfedges[1];
    
    linkHalfedgePair(leftNewFaceHalfedges[0], leftOldFaceHalfedges[2]);
    
    leftOldFace->halfedge = leftOldFaceHalfedges[0];
}

void HalfedgeMesh::breakEdge(HalfedgeMesh::Halfedge *halfedge)
{
    Face *leftOldFace = halfedge->leftFace;
    Halfedge *oppositeHalfedge = halfedge->oppositeHalfedge;
    Face *rightOldFace = nullptr;
    
    if (nullptr != oppositeHalfedge)
        rightOldFace = oppositeHalfedge->leftFace;
    
    Vertex *breakPointVertex = newVertex();
    breakPointVertex->position = (halfedge->startVertex->position +
        halfedge->nextHalfedge->startVertex->position) * 0.5;
        
    //std::cout << "Break first half:" << leftOldFace->debugIndex << " halfedge:" << (int)halfedge << std::endl;
    
    std::vector<Halfedge *> leftNewFaceHalfedges;
    std::vector<Halfedge *> leftOldFaceHalfedges;
    breakFace(leftOldFace, halfedge, breakPointVertex,
        leftNewFaceHalfedges, leftOldFaceHalfedges);
    
    if (nullptr != rightOldFace) {
        //std::cout << "Break second half:" << rightOldFace->debugIndex << " halfedge:" << (int)oppositeHalfedge << std::endl;
        
        std::vector<Halfedge *> rightNewFaceHalfedges;
        std::vector<Halfedge *> rightOldFaceHalfedges;
        breakFace(rightOldFace, oppositeHalfedge, breakPointVertex,
            rightNewFaceHalfedges, rightOldFaceHalfedges);
        linkHalfedgePair(leftOldFaceHalfedges[1], rightNewFaceHalfedges[1]);
        linkHalfedgePair(leftNewFaceHalfedges[1], rightOldFaceHalfedges[1]);
    }
}
