/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: roadparser.cpp
*   Author  : lubing.han
*   Date    : 2016-11-17
*   Describe: Test open dirve parser
*
********************************************************/
#include "roadparser.h"
#include <stdlib.h>
#include <string.h>
#include <string>
#include <iostream>
using namespace std;
using namespace tinyxml2;

namespace opendrive
{

bool process_doc(RoadMap &roadmap, string filepath) {
	XMLDocument doc;
	doc.LoadFile(filepath.c_str());
	XMLElement* root = doc.RootElement();
	if (!root) {
		cerr << "roadparser: Error in Reading file: " << filepath << endl;
		return false;
	}
	XMLElement* rootChild = root->FirstChildElement();
	while (rootChild) {
		const char* rootChildName = rootChild->Name();
		if (strcmp(rootChildName, "road") == 0) {
			XMLElement* road = rootChild;
			const char* len_value = road->Attribute("length");
			double len = atof(len_value);
			const char* id_value = road->Attribute("id");
			const char* junc_value = road->Attribute("junction");
			// if (*id_value) cout << "Road id: " << id_value << endl;
			Road roadclass(len, id_value, junc_value);
			XMLElement* roadchild = road->FirstChildElement();
			while (roadchild) {
				const char* name = roadchild->Name();
				if (strcmp(name, "link") == 0) {
					process_link(&roadclass, roadchild);
				}
				else if (strcmp(name, "planView") == 0) {
					process_planview(&roadclass, roadchild);
				}
				else if (strcmp(name, "lanes") == 0) {
					process_lanes(&roadclass, roadchild);
				}
				else if (strcmp(name, "signals") == 0) {
					process_signals(&roadclass, roadchild);
				}
				else if (strcmp(name, "markers") == 0) {
					process_markers(&roadclass, roadchild);
				}
				else if (strcmp(name, "elevationProfile") == 0) {
					process_elevations(&roadclass, roadchild);
				}
				roadchild = roadchild->NextSiblingElement();
			}
			roadmap.append_road(roadclass);
		}
		else if (strcmp(rootChildName, "junction") == 0) {
			Junction junction = process_junction(rootChild);
			roadmap.append_junction(junction);	
		}
		else if (strcmp(rootChildName, "object") == 0) {
			Obstacle obstacle(Object::fromXML(rootChild));
			roadmap.append_obstacle(obstacle);
		}
		rootChild = rootChild->NextSiblingElement(); 
	}
	return true;
}

bool process_link(Road* road, XMLElement* link) {
	// cout << "  Processing link." << endl;
	XMLElement* linkchild = link->FirstChildElement();
	while (linkchild) {
		const char* name = linkchild->Name();
		const char* type = linkchild->Attribute("elementType");
		if (!type) type = "";
		const char* id = linkchild->Attribute("elementId");
		if (!id) id = "";
		const char* cp = linkchild->Attribute("contactPoint");
		if (!cp) cp = "";

		if (strcmp(name, "predecessor") == 0) {
			Link link(type, id, cp);
			road->set_predecessor(link);
		}
		else if (strcmp(name, "successor") == 0) {
			Link link(type, id, cp);
			road->set_successor(link);
		}
		linkchild = linkchild->NextSiblingElement();
	}
	return true;
}

bool process_planview(Road* road, XMLElement* planview) {
	// cout << "  Processing planview." << endl;
	XMLElement* geometry = planview->FirstChildElement("geometry");
	while (geometry) {
		XMLElement* curve = geometry->FirstChildElement();
		// if (curve) cout << "    Geometry type: " << curve->Name() << endl;
		const char* x_value = geometry->Attribute("x");
		const char* y_value = geometry->Attribute("y");
		const char* hdg_value = geometry->Attribute("hdg");
		const char* len_value = geometry->Attribute("length");
		double x = atof(x_value);
		double y = atof(y_value);
		double hdg = atof(hdg_value);
		double len = atof(len_value);
		Point point(x, y, 0.0);
		const char* name = curve->Name();
		if (strcmp(name, "spiral") == 0) {
			const char* s_value = curve->Attribute("curvStart");
			const char* e_value = curve->Attribute("curvEnd");
			double s = atof(s_value);
			double e = atof(e_value);
			Spiral* spiral = new Spiral(point, len, hdg, s, e);
			road->append_geometry(spiral);
		}
		else if (strcmp(name, "arc") == 0) {
			const char* c_value = curve->Attribute("curvature");
			double c = atof(c_value);
			Arc* arc = new Arc(point, len, hdg, c);
			road->append_geometry(arc);
		}
		else if (strcmp(name, "poly3") == 0) {
			const char* a_value = curve->Attribute("a");
			const char* b_value = curve->Attribute("b");
			const char* c_value = curve->Attribute("c");
			const char* d_value = curve->Attribute("d");
			double a = atof(a_value);
			double b = atof(b_value);
			double c = atof(c_value);
			double d = atof(d_value);
			Poly3* poly3 = new Poly3(point, len, hdg, a, b, c, d);
			road->append_geometry(poly3);
		}
		else if (strcmp(name, "paramPoly3") == 0) {
			const char* au_value = curve->Attribute("aU");
			const char* bu_value = curve->Attribute("bU");
			const char* cu_value = curve->Attribute("cU");
			const char* du_value = curve->Attribute("dU");
			const char* av_value = curve->Attribute("aV");
			const char* bv_value = curve->Attribute("bV");
			const char* cv_value = curve->Attribute("cV");
			const char* dv_value = curve->Attribute("dV");
			double au = atof(au_value);
			double bu = atof(bu_value);
			double cu = atof(cu_value);
			double du = atof(du_value);
			double av = atof(av_value);
			double bv = atof(bv_value);
			double cv = atof(cv_value);
			double dv = atof(dv_value);
			ParamPoly3* parampoly3 = new ParamPoly3(point, len, hdg, au, bu, cu, du, av, bv, cv, dv);
			road->append_geometry(parampoly3);
		}
		else {
			Line* line = new Line(point, len, hdg);
			road->append_geometry(line);
		}
		geometry = geometry->NextSiblingElement("geometry");
	}
	return true;
}

bool process_lanes(Road* road, XMLElement* lanes){
		XMLElement* lane_offset = lanes->FirstChildElement("laneOffset");
    while (lane_offset) {
        double s = atof(lane_offset->Attribute("s"));
        double a = atof(lane_offset->Attribute("a"));
        double b = atof(lane_offset->Attribute("b"));
        double c = atof(lane_offset->Attribute("c"));
        double d = atof(lane_offset->Attribute("d"));
        road->append_lane_offset(LaneOffset(s, a, b, c, d));

        lane_offset = lane_offset->NextSiblingElement("laneOffset");
    }

		XMLElement* section = lanes->FirstChildElement("laneSection");
    while (section) {
        double sec_offset = atof(section->Attribute("s"));
        LaneSection sec;
        sec.set_sOffset(sec_offset);
        // left lanes
        XMLElement* left = section->FirstChildElement("left");
        if (left) {
            left = left->FirstChildElement("lane");
            while (left) {
                Lane lane_t;
                process_lane(lane_t, left);
                if (lane_t.use_widths())
                	sec.set_use_widths(true);
                sec.append_lane(lane_t);
                left = left->NextSiblingElement("lane");
            }
        }

        // center lane
        XMLElement* center = section->FirstChildElement("center");
        if (center) {
            center = center->FirstChildElement("lane");
            while (center) {
                Lane lane_t;
                process_lane(lane_t, center);
                if (lane_t.use_widths())
                	sec.set_use_widths(true);
                sec.append_lane(lane_t);
                center = center->NextSiblingElement("lane");
            }
        }
        
        // right lanes
        XMLElement* right = section->FirstChildElement("right");
        if (right) {
            right = right->FirstChildElement("lane");
            while (right) {
                Lane lane_t;
                process_lane(lane_t, right);
                if (lane_t.use_widths())
                	sec.set_use_widths(true);
                sec.append_lane(lane_t);
                right = right->NextSiblingElement("lane");
            }
        }
        road->append_section(sec);
        section = section->NextSiblingElement("laneSection");
    }
		return true;
}

bool process_lane(Lane& lane, XMLElement* ele) {
    lane.set_id( atoi(ele->Attribute("id")) );
    
    lane.set_type( ele->Attribute("type") );

    // cout << "current Lane :" <<  ele->Attribute("id") << "current type: " << ele->Attribute("type") << endl;

    lane.set_level( (bool) atoi(ele->Attribute("level")) );
    XMLElement* link = ele->FirstChildElement("link");
    if (link) {
        XMLElement* pre = link->FirstChildElement("predecessor");
        if (pre) {
            lane.set_predecessor( atoi(pre->Attribute("id")) );
        }

        XMLElement* suc = link->FirstChildElement("successor");
        if (suc) {
            lane.set_successor( atoi(suc->Attribute("id")) );
        }
    }

    // extract width
    XMLElement* width = ele->FirstChildElement("width");
    while (width) {
        Polynomial p(
                atof(width->Attribute("sOffset")),
                atof(width->Attribute("a")),
                atof(width->Attribute("b")),
                atof(width->Attribute("c")),
                atof(width->Attribute("d"))
        );
        lane.append_width(p);
        width = width->NextSiblingElement("width");
    }

    XMLElement* border = ele->FirstChildElement("border");
    while (border) {
        Polynomial p(
                atof(border->Attribute("sOffset")),
                atof(border->Attribute("a")),
                atof(border->Attribute("b")),
                atof(border->Attribute("c")),
                atof(border->Attribute("d"))
        );
        lane.append_border(p);
        border = border->NextSiblingElement("border");
    }

    // extract roadmark
    XMLElement* mark = ele->FirstChildElement("roadMark");
    while (mark) {
        RoadMark m;
				const char* sOffset = mark->Attribute("sOffset");
				const char* type = mark->Attribute("type");
				const char* weight = mark->Attribute("weight");
				const char* color = mark->Attribute("color");
				const char* material = mark->Attribute("material");
				const char* width = mark->Attribute("width");
				const char* laneChange = mark->Attribute("laneChange");
				const char* height = mark->Attribute("height");
				if (sOffset) m.set_sOffset(atof(sOffset));
				if (type) m.set_type(type);
				if (weight) m.set_weight(weight);
				if (color) m.set_color(color);
				if (material) m.set_material(material);
				if (width) m.set_width(atof(width));
				if (laneChange) m.set_laneChange(laneChange);
				if (height) m.set_height(atof(height));
        lane.append_roadmark(m);
        mark = mark->NextSiblingElement("roadMark");
    }
	return true;
}

bool process_signals(Road* road, XMLElement* signals) {
	XMLElement* signal = signals->FirstChildElement();
	while (signal) {
		const char* signal_type = signal->Name();
		Signal sig;
		if (strcmp(signal_type, "signal") == 0) {
			sig.set_isReference(false);
			const char* s = signal->Attribute("s");
			const char* t = signal->Attribute("t");
			const char* dx = signal->Attribute("dx");
			const char* dy = signal->Attribute("dy");
			const char* id = signal->Attribute("id");
			const char* name = signal->Attribute("name");
			const char* dynamic = signal->Attribute("dynamic");
			const char* orientation = signal->Attribute("orientation");
			const char* zOffset = signal->Attribute("zOffset");
			const char* country = signal->Attribute("country");
			const char* type = signal->Attribute("type");
			const char* subtype = signal->Attribute("subtype");
			const char* value = signal->Attribute("value");
			const char* unit = signal->Attribute("unit");
			const char* height = signal->Attribute("height");
			const char* width = signal->Attribute("width");
			const char* text = signal->Attribute("text");
			const char* hOffset = signal->Attribute("hOffset");
			const char* pitch = signal->Attribute("pitch");
			const char* roll = signal->Attribute("roll");
			// if (!(s && t && id)) cerr << "roadparser reading error in process_signals" << endl;
			if (s) sig.set_s(atof(s));
			if (t) sig.set_t(atof(t));
			if (dx) sig.set_dx(atof(dx));
			if (dy) sig.set_dy(atof(dy));
			if (id) sig.set_id(id);
			if (name) sig.set_name(name);
			if (dynamic) sig.set_dynamic(dynamic);
			if (orientation) sig.set_orientation(orientation);
			if (zOffset) sig.set_zOffset(atof(zOffset));
			if (country) sig.set_country(country);
			if (type) sig.set_type(type);
			if (subtype) sig.set_subtype(subtype);
			if (value) sig.set_value(atof(value));
			if (unit) sig.set_unit(unit);
			if (height) sig.set_height(atof(height));
			if (width) sig.set_width(atof(width));
			if (text) sig.set_text(text);
			if (hOffset) sig.set_hOffset(atof(hOffset));
			if (pitch) sig.set_pitch(atof(pitch));
			if (roll) sig.set_roll(atof(roll));
			XMLElement* signalChild = signal->FirstChildElement();
			while (signalChild) {
				const char* childName = signalChild->Name();
				if (strcmp(childName, "validity") == 0) {
					Validity validity;
					const char* fromLane = signalChild->Attribute("fromLane");
					const char* toLane = signalChild->Attribute("toLane");
					if (fromLane) validity.set_fromLane(atoi(fromLane));
					if (toLane) validity.set_toLane(atoi(toLane));
					sig.append_validity(validity);
				}
				else if (strcmp(childName, "dependency") == 0) {
					Dependency dependency;
					const char* id = signalChild->Attribute("id");
					const char* type = signalChild->Attribute("type");
					if (id) dependency.set_id(id);
					if (type) dependency.set_type(type);
					sig.append_dependency(dependency);
				}
				signalChild = signalChild->NextSiblingElement();
			}
		}
		else if (strcmp(signal_type, "signalReference") == 0) {
			sig.set_isReference(true);
			const char* s = signal->Attribute("s");
			const char* t = signal->Attribute("t");
			const char* id = signal->Attribute("id");
			const char* orientation = signal->Attribute("orientation");
			if (s) sig.set_s(atof(s));
			if (t) sig.set_t(atof(t));
			if (id) sig.set_id(id);
			if (orientation) sig.set_orientation(orientation);
			if (!(s && t && id)) cerr << "roadparser reading error in process_signals" << endl;
			XMLElement* signalChild = signal->FirstChildElement();
			while (signalChild) {
				const char* childName = signalChild->Name();
				if (strcmp(childName, "validity") == 0) {
					Validity validity;
					const char* fromLane = signalChild->Attribute("fromLane");
					const char* toLane = signalChild->Attribute("toLane");
					if (fromLane) validity.set_fromLane(atoi(fromLane));
					if (toLane) validity.set_toLane(atoi(toLane));
					sig.append_validity(validity);
				}
				signalChild = signalChild->NextSiblingElement();
			}
		}
		// sig.print();
		road->append_signal(sig);
		signal = signal->NextSiblingElement();
	}
	return true;
}

bool process_markers(Road* road, XMLElement* markers) {
	XMLElement* marker = markers->FirstChildElement();
	while (marker) {
		Marker mak;
		const char* s = marker->Attribute("s");
		const char* t = marker->Attribute("t");
		const char* id = marker->Attribute("id");
		const char* name = marker->Attribute("name");
		const char* height = marker->Attribute("height");
		const char* zOffset = marker->Attribute("zOffset");
		const char* type = marker->Attribute("type");
		if (!(s && t && id)) cerr << "roadparser reading error in process_markers" << endl;
		if (s) mak.set_s(atof(s));
		if (t) mak.set_t(atof(t));
		if (id) mak.set_id(id);
		if (name) mak.set_name(name);
		if (height) mak.set_height(atof(height));
		if (zOffset) mak.set_zOffset(atof(zOffset));
		if (type) mak.set_type(type);
		XMLElement* markerChild = marker->FirstChildElement();
		while (markerChild) {
			const char* childName = markerChild->Name();
			if (strcmp(childName, "validity") == 0) {
				Validity validity;
				const char* fromLane = markerChild->Attribute("fromLane");
				const char* toLane = markerChild->Attribute("toLane");
				if (fromLane) validity.set_fromLane(atoi(fromLane));
				if (toLane) validity.set_toLane(atoi(toLane));
				mak.append_validity(validity);
			}
			else if (strcmp(childName, "outline") == 0) {
				Outline outline;
				XMLElement* outlineChild = markerChild->FirstChildElement();
				while (outlineChild) {
					const char* outlineChildName = outlineChild->Name();
					if (strcmp(outlineChildName, "cornerRoad") == 0) {
						CornerRoad cornerRoad;
						const char* s = outlineChild->Attribute("s");
						const char* t = outlineChild->Attribute("t");
						const char* dz = outlineChild->Attribute("dz");
						const char* height = outlineChild->Attribute("height");
						if (s) cornerRoad.set_s(atof(s));
						if (t) cornerRoad.set_t(atof(t));
						if (dz) cornerRoad.set_dz(atof(dz));
						if (height) cornerRoad.set_height(atof(height));
						outline.append_cornerRoad(cornerRoad);
					}
					outlineChild = outlineChild->NextSiblingElement();
				}
				mak.set_outline(outline);
			}
			else if (strcmp(childName, "reference") == 0) {
				Reference reference;
				const char* signalId = markerChild->Attribute("signalId");
				const char* laneId = markerChild->Attribute("laneId");
				if (signalId) reference.set_signalId(signalId);
				if (laneId) reference.set_laneId(atoi(laneId));
				mak.append_reference(reference);
			}
			markerChild = markerChild->NextSiblingElement();
		}
		// mak.print();
		road->append_marker(mak);
		marker = marker->NextSiblingElement();
	}
	return true;
}

bool process_elevations(Road* road, XMLElement* elevations) {
	XMLElement* elevation = elevations->FirstChildElement();
	while (elevation) {
		Elevation ele;
		const char* s = elevation->Attribute("s");
		const char* a = elevation->Attribute("a");
		const char* b = elevation->Attribute("b");
		const char* c = elevation->Attribute("c");
		const char* d = elevation->Attribute("d");
		if (s) ele.set_s(atof(s));
		if (a) ele.set_a(atof(a));
		if (b) ele.set_b(atof(b));
		if (c) ele.set_c(atof(c));
		if (d) ele.set_d(atof(d));
		road->append_elevation(ele);
		elevation = elevation->NextSiblingElement();
	}
	return true;
}

Junction process_junction(XMLElement* ele) {
	const char* name = ele->Attribute("name");
	const char* id = ele->Attribute("id");
	if (!name) name = "";
	if (!id) id = "";
	Junction junction(name, id);
	XMLElement* eleChild = ele->FirstChildElement();
	while (eleChild) {
		const char* childName = eleChild->Name();
		if (strcmp(childName, "connection") == 0)
			junction.append_connection(process_connection(eleChild));
		else if (strcmp(childName, "priority") == 0)
			junction.append_priority(process_priority(eleChild));
		else if (strcmp(childName, "controller") == 0)
			junction.append_junctionController(process_junctionController(eleChild));
		eleChild = eleChild->NextSiblingElement();
	}
	return junction;
}

Connection process_connection(XMLElement* ele) {
	const char* id = ele->Attribute("id");
	const char* incomingRoad = ele->Attribute("incomingRoad");
	const char* connectingRoad = ele->Attribute("connectingRoad");
	const char* contactPoint = ele->Attribute("contactPoint");
	if (!id) id = "";
	if (!incomingRoad) incomingRoad = "";
	if (!connectingRoad) connectingRoad = "";
	if (!contactPoint) contactPoint = "";
	Connection connection(id, incomingRoad, connectingRoad, contactPoint);
	XMLElement* eleChild = ele->FirstChildElement("laneLink");
	while (eleChild) {
		const char* from_name = eleChild->Attribute("from");
		const char* to_name = eleChild->Attribute("to");
		if (!from_name || !to_name) continue;
		LaneLink laneLink = LaneLink(atoi(from_name), atoi(to_name));
		connection.append_laneLink(laneLink);
		eleChild = eleChild->NextSiblingElement();
	}
	return connection;
}

Priority process_priority(XMLElement* ele) {
	const char* high = ele->Attribute("high");
	const char* low = ele->Attribute("lo");
	if (!high) high = "";
	if (!low) low = "";
	Priority priority(high, low);
	return priority;
}

JunctionController process_junctionController(XMLElement* ele) {
	const char* id = ele->Attribute("id");
	const char* type = ele->Attribute("type");
	const char* sequence = ele->Attribute("sequence");
	if (!id) id = "";
	if (!type) type = "";
	if (!sequence) sequence = "";
	JunctionController jc = JunctionController(id, type, sequence);
	return jc;
}
    
}
