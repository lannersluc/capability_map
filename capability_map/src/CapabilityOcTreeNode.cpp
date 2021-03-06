// Copyright (c) 2014, Jochen Kempfle
// All rights reserved.

/*
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/


#include <cmath>
#include "capability_map/CapabilityOcTreeNode.h"


bool Capability::isDirectionPossible(double phi, double theta) const
{
    assert(theta <= 180.0);

    // convert angles to rad
    double thetaInRad1 = M_PI * (90.0 - _theta) / 180.0;
    double thetaInRad2 = M_PI * (90.0 - theta) / 180.0;
    double phiInRad1 = M_PI * _phi / 180.0;
    double phiInRad2 = M_PI * phi / 180.0;
    // calculate the shortest angle (great-circle distance) between both directions
    double orthodromeAngle = acos(sin(thetaInRad1) * sin(thetaInRad2) + cos(thetaInRad1) *
                                  cos(thetaInRad2) * cos(phiInRad2 - phiInRad1)) * 180.0 / M_PI;

    bool retValue = false;

    switch (_type)
    {
        case EMPTY:
            // no direction possible
            retValue = false;
            break;

        case SPHERE:
            // every direction possible
            retValue = true;
            break;

        case CONE:
            // only directions are possible that lie inside the cone's opening angle
            retValue = orthodromeAngle <= _halfOpeningAngle;
            break;

        case CYLINDER_1:
            // CYLINDER_1 can be seen as double cone, possible directions must lie inside opening angle of both sides
            retValue = orthodromeAngle <= _halfOpeningAngle || std::abs(orthodromeAngle - 180.0) <= _halfOpeningAngle;
            break;

        case CYLINDER_2:
            // the sensitive area of the cylinder is orthogonal to the up-direction (up-direction +- 90°)
            retValue = std::abs(90.0 - orthodromeAngle) <= _halfOpeningAngle;
            break;

        default:
            retValue = false;
    }

    return retValue;
}

double Capability::getPercentReachable() const
{
    double retValue;

    switch (_type)
    {
        case EMPTY:
            // no direction possible
            retValue = 0.0;
            break;

        case SPHERE:
            // every direction possible
            retValue = 100.0;
            break;

        case CONE:
            // area of CONE is: 2 * PI * (1 - cos(halfOpeningAngle)). With sphere area = 4 * PI * r^2 it simplifies to:
            retValue = 100.0 * (1.0 - cos(_halfOpeningAngle * M_PI / 180.0)) / 2.0;
            break;

        case CYLINDER_1:
            // CYLINDER_1 can be seen as double cone, multiply above formula by 2:
            retValue = 100.0 * (1.0 - cos(_halfOpeningAngle * M_PI / 180.0));
            break;

        case CYLINDER_2:
            // area of CYLINDER_2 is: 2 * PI * r * h. h is: 2 * sin(halfOpeningAngle)
            retValue = 100.0 * sin(_halfOpeningAngle * M_PI / 180.0);
            break;

        default:
            retValue = 0.0;
    }

    return retValue;
}

CapabilityOcTreeNode::CapabilityOcTreeNode() : OcTreeDataNode<Capability>(Capability(EMPTY, 0.0, 0.0, 0.0, 0.0))
{

}

CapabilityOcTreeNode::CapabilityOcTreeNode(Capability capability) : OcTreeDataNode<Capability>(capability)
{

}

CapabilityOcTreeNode::~CapabilityOcTreeNode()
{

}

bool CapabilityOcTreeNode::createChild(unsigned int i)
{
    if (children == NULL)
    {
        allocChildren();
    }
    assert (children[i] == NULL);
    children[i] = new CapabilityOcTreeNode();
    return true;
}


std::ostream& CapabilityOcTreeNode::writeValue(std::ostream &s) const
{
    // 1 bit for each children; 0: empty, 1: allocated
    std::bitset<8> children;
    for (unsigned int i = 0; i < 8; ++i)
    {
        if (childExists(i))
        {
          children[i] = 1;
        }
        else
        {
          children[i] = 0;
        }
    }
    char children_char = (char)children.to_ulong();

    // buffer capabilities data
    unsigned char type = value.getType();
    double phi = value.getPhi();
    double theta = value.getTheta();
    double halfOpeningAngle = value.getHalfOpeningAngle();
    double SFE = value.getShapeFitError();

    // write node data
    s.write((const char*)&type, sizeof(char));
    s.write((const char*)&phi, sizeof(double));
    s.write((const char*)&theta, sizeof(double));
    s.write((const char*)&halfOpeningAngle, sizeof(double));
    s.write((const char*)&SFE, sizeof(double));
    s.write((char*)&children_char, sizeof(char)); // child existence

    // write existing children
    for (unsigned int i = 0; i < 8; ++i)
    {
        if (children[i] == 1)
        {
            this->getChild(i)->writeValue(s);
        }
    }
    return s;
}


std::istream& CapabilityOcTreeNode::readValue(std::istream &s)
{
    // buffer for capabilities' data
    unsigned char type;
    double phi;
    double theta;
    double halfOpeningAngle;
    double SFE;

    // read node data
    char children_char;
    s.read((char*)&type, sizeof(char));
    s.read((char*)&phi, sizeof(double));
    s.read((char*)&theta, sizeof(double));
    s.read((char*)&halfOpeningAngle, sizeof(double));
    s.read((char*)&SFE, sizeof(double));
    s.read((char*)&children_char, sizeof(char)); // child existence

    // insert buffered data into node
    value.setType(type);
    value.setDirection(phi, theta);
    value.setHalfOpeningAngle(halfOpeningAngle);
    value.setShapeFitError(SFE);

    // read existing children
    std::bitset<8> children ((unsigned long long)children_char);
    for (unsigned int i = 0; i < 8; ++i)
    {
        if (children[i] == 1)
        {
            createChild(i);
            getChild(i)->readValue(s);
        }
    }
    return s;
}


