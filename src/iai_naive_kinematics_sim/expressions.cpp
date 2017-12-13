#include "iai_naive_kinematics_sim/expressions.h"
#include "iai_naive_kinematics_sim/simulator.hpp"

#include <iostream>

namespace iai_naive_kinematics_sim {

	bool ExpressionTree::parseYAML(const YAML::Node& root,
		   unordered_map<size_t, Expression<double>*>& posExprs,
		   unordered_map<size_t, Expression<double>*>& velExprs,
		   unordered_map<size_t, Expression<double>*>& effExprs) {

		if (root.IsSequence()) {

			for (size_t i = 0; i < root.size(); i++)
			{
				if (!root[i].IsMap()) {
					cerr << "Fake controllers need to be a map! Node: " << endl << root[i] << endl;
					return false;
				}

				try {
					string jointName = root[i].begin()->first.as<string>();
					if (sim->hasJoint(jointName))
					{
						size_t idx = sim->getJointIndex(jointName);
						if (root[i].begin()->second["position"]) {
							Expression<double>* posExp = parseDoubleExpr(root[i].begin()->second["position"]);
							if (posExp) {
								posExprs[idx] = posExp;
							} else {
								cerr << "Parsing of position expression for joint '" << jointName << "' failed! Node: " << endl << root[i].begin()->second << endl;
								return false;
							}
						}

						if (root[i].begin()->second["velocitiy"]) {
							Expression<double>* velExp = parseDoubleExpr(root[i].begin()->second["velocitiy"]);
							if (velExp) {
								velExprs[idx] = velExp;
							} else {
								cerr << "Parsing of velocitiy expression for joint '" << jointName << "' failed! Node: " << endl << root[i].begin()->second << endl;
								return false;
							}
						}

						if (root[i].begin()->second["effort"]) {
							Expression<double>* effExp = parseDoubleExpr(root[i].begin()->second["effort"]);
							if (effExp) {
								effExprs[idx] = effExp;
							} else {
								cerr << "Parsing of effort expression for joint '" << jointName << "' failed! Node: " << endl << root[i].begin()->second << endl;
								return false;
							}
						}
					} else {
						cerr << "Joint of name '" << jointName << "' is unknown. Aborting parse!" << endl;
						return false;
					}
				} catch (const YAML::Exception& e) {
					cerr << "Unable to parse node as string! Node: " << endl << root[i] << endl;
				}
			}
		} else {
			cerr << "Fake controllers node needs to be a sequence!" << endl;
			return false;
		}

		return true;
	}

	Expression<double>* ExpressionTree::parseDoubleExpr(const YAML::Node& node){
		switch(node.Type()) {
			case YAML::NodeType::Scalar:
			{
				boost::shared_ptr<ConstDoubleExpr> ptr(new ConstDoubleExpr(node.as<double>()));
				doubleExpressions.push_back(ptr);
				return ptr.get();
			}
			break;
			case YAML::NodeType::Map:
			{
				if (node.size() == 1)
				{
					auto it = node.begin();
					string key = it->first.as<string>();
					Expression<double>* out = 0;
					if (key.compare("pos-of") == 0) {
						out = parsePositionExpr(it->second);
					} else if (key.compare("vel-of") == 0) {
						out = parseVelocityExpr(it->second);
					} else if (key.compare("eff-of") == 0) {
						out = parseEffortExpr(it->second);
					} else if (key.compare("f-pos-of") == 0) {
						out = parsePositionFracExpr(it->second);
					} else if (key.compare("f-vel-of") == 0) {
						out = parseVelocityFracExpr(it->second);
					} else if (key.compare("f-eff-of") == 0) {
						out = parseEffortFracExpr(it->second);
					} else if (key.compare("pos-lim-low-of") == 0) {
						out = parsePosLowLimitExpr(it->second);
					} else if (key.compare("pos-lim-hig-of") == 0) {
						out = parsePosUpLimitExpr(it->second);
					} else if (key.compare("pos-lim-len-of") == 0) {
						out = parsePosLimitSpreadExpr(it->second);
					} else if (key.compare("vel-lim-of") == 0) {
						out = parseVelocityLimitExpr(it->second);
					} else if (key.compare("eff-lim-of") == 0) {
						out = parseEffortLimitExpr(it->second);
					} else if (key.compare("add") == 0) {
						out = parseAddExpr(it->second);
					} else if (key.compare("sub") == 0) {
						out = parseSubExpr(it->second);
					} else if (key.compare("mul") == 0) {
						out = parseMulExpr(it->second);
					} else if (key.compare("div") == 0) {
						out = parseDivExpr(it->second);
					} else if (key.compare("min") == 0) {
						out = parseMinExpr(it->second);
					} else if (key.compare("max") == 0) {
						out = parseMaxExpr(it->second);
					} else if (key.compare("sin") == 0) {
						out = parseSinExpr(it->second);
					} else if (key.compare("cos") == 0) {
						out = parseCosExpr(it->second);
					} else if (key.compare("abs") == 0) {
						out = parseAbsExpr(it->second);
					} else {
						cerr << "'" << key << "' is not a valid name for a double expression! Node: " << endl << node << endl;
					}

					if (!out)
					{
						cerr << "Parsing of double node failed! Node:" << endl << node << endl;
					}
					return out;
				} else {
					cerr << "Only yaml-maps of length 1 can be translated into double expressions! Node: " << endl << node << endl;
				}
				break;
			}
			case YAML::NodeType::Undefined:
			{
				string name = node.as<string>();
				auto it = namedDoubleExpr.find(name);
				if (it != namedDoubleExpr.end())
				{
					return it->second;
				} else {
					cerr << "Undefined expression '" << name << "'" << endl;
				}
				break;
			}
			default:
			cerr << "Node type unsupported for double expressions! Node: " << endl << node << endl;
			break;
		}

		return 0;
	}

	AddExpr* ExpressionTree::parseAddExpr(const YAML::Node& node){
		if (!node.IsSequence() || node.size() != 2)
			return 0;

		Expression<double>* a, *b;
		a = parseDoubleExpr(node[0]);
		b = parseDoubleExpr(node[1]);

		if (!a || !b)
		{
			cerr << "Parsing of Add-expression failed! Node:" << endl << node << endl;
			return 0;
		}

		boost::shared_ptr<AddExpr> ptr(new AddExpr(a, b));
		doubleExpressions.push_back(ptr);

		return ptr.get();
	}

	SubExpr* ExpressionTree::parseSubExpr(const YAML::Node& node){
		if (!node.IsSequence() || node.size() != 2)
			return 0;

		Expression<double>* a, *b;
		a = parseDoubleExpr(node[0]);
		b = parseDoubleExpr(node[1]);

		if (!a || !b)
		{
			cerr << "Parsing of Sub-expression failed! Node:" << endl << node << endl;
			return 0;
		}

		boost::shared_ptr<SubExpr> ptr(new SubExpr(a, b));
		doubleExpressions.push_back(ptr);

		return ptr.get();
	}

	MulExpr* ExpressionTree::parseMulExpr(const YAML::Node& node){
		if (!node.IsSequence() || node.size() != 2)
			return 0;

		Expression<double>* a, *b;
		a = parseDoubleExpr(node[0]);
		b = parseDoubleExpr(node[1]);

		if (!a || !b)
		{
			cerr << "Parsing of Mul-expression failed! Node:" << endl << node << endl;
			return 0;
		}

		boost::shared_ptr<MulExpr> ptr(new MulExpr(a, b));
		doubleExpressions.push_back(ptr);

		return ptr.get();
	}

	DivExpr* ExpressionTree::parseDivExpr(const YAML::Node& node){
		if (!node.IsSequence() || node.size() != 2)
			return 0;

		Expression<double>* a, *b;
		a = parseDoubleExpr(node[0]);
		b = parseDoubleExpr(node[1]);

		if (!a || !b)
		{
			cerr << "Parsing of Div-expression failed! Node:" << endl << node << endl;
			return 0;
		}

		boost::shared_ptr<DivExpr> ptr(new DivExpr(a, b));
		doubleExpressions.push_back(ptr);

		return ptr.get();
	}

	MinExpr* ExpressionTree::parseMinExpr(const YAML::Node& node){
		if (!node.IsSequence() || node.size() != 2)
			return 0;

		Expression<double>* a, *b;
		a = parseDoubleExpr(node[0]);
		b = parseDoubleExpr(node[1]);

		if (!a || !b)
		{
			cerr << "Parsing of Min-expression failed! Node:" << endl << node << endl;
			return 0;
		}

		boost::shared_ptr<MinExpr> ptr(new MinExpr(a, b));
		doubleExpressions.push_back(ptr);

		return ptr.get();
	}

	MaxExpr* ExpressionTree::parseMaxExpr(const YAML::Node& node){
		if (!node.IsSequence() || node.size() != 2)
			return 0;

		Expression<double>* a, *b;
		a = parseDoubleExpr(node[0]);
		b = parseDoubleExpr(node[1]);

		if (!a || !b)
		{
			cerr << "Parsing of Max-expression failed! Node:" << endl << node << endl;
			return 0;
		}

		boost::shared_ptr<MaxExpr> ptr(new MaxExpr(a, b));
		doubleExpressions.push_back(ptr);

		return ptr.get();
	}

	AbsExpr* ExpressionTree::parseAbsExpr(const YAML::Node& node){
		Expression<double>* a;
		a = parseDoubleExpr(node[0]);

		if (!a)
		{
			cerr << "Parsing of Abs-expression failed! Node:" << endl << node << endl;
			return 0;
		}

		boost::shared_ptr<AbsExpr> ptr(new AbsExpr(a));
		doubleExpressions.push_back(ptr);

		return ptr.get();
	}

	SinExpr* ExpressionTree::parseSinExpr(const YAML::Node& node){
		Expression<double>* a;
		a = parseDoubleExpr(node[0]);

		if (!a)
		{
			cerr << "Parsing of Sin-expression failed! Node:" << endl << node << endl;
			return 0;
		}

		boost::shared_ptr<SinExpr> ptr(new SinExpr(a));
		doubleExpressions.push_back(ptr);

		return ptr.get();
	}

	CosExpr* ExpressionTree::parseCosExpr(const YAML::Node& node){
		Expression<double>* a;
		a = parseDoubleExpr(node[0]);

		if (!a)
		{
			cerr << "Parsing of Cos-expression failed! Node:" << endl << node << endl;
			return 0;
		}

		boost::shared_ptr<CosExpr> ptr(new CosExpr(a));
		doubleExpressions.push_back(ptr);

		return ptr.get();
	}


	PositionExpr* 	ExpressionTree::parsePositionExpr(const YAML::Node& node){
		try{
			string jointName = node.as<string>();
			if (sim->hasJoint(jointName)) {
				boost::shared_ptr<PositionExpr> ptr(new PositionExpr(sim->state_, sim->getJointIndex(jointName)));
				doubleExpressions.push_back(ptr);
				return ptr.get();
			} else
			cerr << "Unknown joint '" << jointName << "' while parsing position expression! Node:" << endl << node << endl;

		} catch (const YAML::Exception& e) {
			cerr << "Parsing of joint-name failed! Node:" << endl << node << endl;
			return 0;
		}

		return 0;
	}

	VelocityExpr* 	ExpressionTree::parseVelocityExpr(const YAML::Node& node){
		try{
			string jointName = node.as<string>();
			if (sim->hasJoint(jointName)) {
				boost::shared_ptr<VelocityExpr> ptr(new VelocityExpr(sim->state_, sim->getJointIndex(jointName)));
				doubleExpressions.push_back(ptr);
				return ptr.get();
			} else
			cerr << "Unknown joint '" << jointName << "' while parsing Velocity expression! Node:" << endl << node << endl;

		} catch (const YAML::Exception& e) {
			cerr << "Parsing of joint-name failed! Node:" << endl << node << endl;
			return 0;
		}
		return 0;
	}

	EffortExpr* 	ExpressionTree::parseEffortExpr(const YAML::Node& node){
		try{
			string jointName = node.as<string>();
			if (sim->hasJoint(jointName)) {
				boost::shared_ptr<EffortExpr> ptr(new EffortExpr(sim->state_, sim->getJointIndex(jointName)));
				doubleExpressions.push_back(ptr);
				return ptr.get();
			} else
			cerr << "Unknown joint '" << jointName << "' while parsing Effort expression! Node:" << endl << node << endl;

		} catch (const YAML::Exception& e) {
			cerr << "Parsing of joint-name failed! Node:" << endl << node << endl;
			return 0;
		}
		return 0;
	}


	PositionFracExpr* 	ExpressionTree::parsePositionFracExpr(const YAML::Node& node){
		try{
			string jointName = node.as<string>();
			if (sim->hasJoint(jointName)) {
				boost::shared_ptr<PositionFracExpr> ptr(new PositionFracExpr(sim->state_, sim->getJointIndex(jointName), sim->getJoint(jointName)->limits));
				doubleExpressions.push_back(ptr);
				return ptr.get();
			} else
			cerr << "Unknown joint '" << jointName << "' while parsing positionFrac expression! Node:" << endl << node << endl;

		} catch (const YAML::Exception& e) {
			cerr << "Parsing of joint-name failed! Node:" << endl << node << endl;
			return 0;
		}
		return 0;
	}

	VelocityFracExpr* 	ExpressionTree::parseVelocityFracExpr(const YAML::Node& node){
		try{
			string jointName = node.as<string>();
			if (sim->hasJoint(jointName)) {
				boost::shared_ptr<VelocityFracExpr> ptr(new VelocityFracExpr(sim->state_, sim->getJointIndex(jointName), sim->getJoint(jointName)->limits));
				doubleExpressions.push_back(ptr);
				return ptr.get();
			} else
			cerr << "Unknown joint '" << jointName << "' while parsing VelocityFrac expression! Node:" << endl << node << endl;

		} catch (const YAML::Exception& e) {
			cerr << "Parsing of joint-name failed! Node:" << endl << node << endl;
			return 0;
		}
		return 0;
	}

	EffortFracExpr* 	ExpressionTree::parseEffortFracExpr(const YAML::Node& node){
		try{
			string jointName = node.as<string>();
			if (sim->hasJoint(jointName)) {
				boost::shared_ptr<EffortFracExpr> ptr(new EffortFracExpr(sim->state_, sim->getJointIndex(jointName), sim->getJoint(jointName)->limits));
				doubleExpressions.push_back(ptr);
				return ptr.get();
			} else
			cerr << "Unknown joint '" << jointName << "' while parsing EffortFrac expression! Node:" << endl << node << endl;

		} catch (const YAML::Exception& e) {
			cerr << "Parsing of joint-name failed! Node:" << endl << node << endl;
			return 0;
		}
		return 0;
	}


	PosLowLimitExpr* 	ExpressionTree::parsePosLowLimitExpr(const YAML::Node& node){
		try{
			string jointName = node.as<string>();
			if (sim->hasJoint(jointName)) {
				boost::shared_ptr<PosLowLimitExpr> ptr(new PosLowLimitExpr(sim->getJoint(jointName)->limits));
				doubleExpressions.push_back(ptr);
				return ptr.get();
			} else
			cerr << "Unknown joint '" << jointName << "' while parsing PosLowLimit expression! Node:" << endl << node << endl;

		} catch (const YAML::Exception& e) {
			cerr << "Parsing of joint-name failed! Node:" << endl << node << endl;
			return 0;
		}
		return 0;
	}

	PosUpLimitExpr* 	ExpressionTree::parsePosUpLimitExpr(const YAML::Node& node){
		try{
			string jointName = node.as<string>();
			if (sim->hasJoint(jointName)) {
				boost::shared_ptr<PosUpLimitExpr> ptr(new PosUpLimitExpr(sim->getJoint(jointName)->limits));
				doubleExpressions.push_back(ptr);
				return ptr.get();
			} else
			cerr << "Unknown joint '" << jointName << "' while parsing PosUpLimit expression! Node:" << endl << node << endl;

		} catch (const YAML::Exception& e) {
			cerr << "Parsing of joint-name failed! Node:" << endl << node << endl;
			return 0;
		}
		return 0;
	}

	PosLimitSpreadExpr* ExpressionTree::parsePosLimitSpreadExpr(const YAML::Node& node){
		try{
			string jointName = node.as<string>();
			if (sim->hasJoint(jointName)) {
				boost::shared_ptr<PosLimitSpreadExpr> ptr(new PosLimitSpreadExpr(sim->getJoint(jointName)->limits));
				doubleExpressions.push_back(ptr);
				return ptr.get();
			} else
			cerr << "Unknown joint '" << jointName << "' while parsing PosLimitSpread expression! Node:" << endl << node << endl;

		} catch (const YAML::Exception& e) {
			cerr << "Parsing of joint-name failed! Node:" << endl << node << endl;
			return 0;
		}
		return 0;
	}


	VelocityLimitExpr* 	ExpressionTree::parseVelocityLimitExpr(const YAML::Node& node){
		try{
			string jointName = node.as<string>();
			if (sim->hasJoint(jointName)) {
				boost::shared_ptr<VelocityLimitExpr> ptr(new VelocityLimitExpr(sim->getJoint(jointName)->limits));
				doubleExpressions.push_back(ptr);
				return ptr.get();
			} else
			cerr << "Unknown joint '" << jointName << "' while parsing VelocityLimit expression! Node:" << endl << node << endl;

		} catch (const YAML::Exception& e) {
			cerr << "Parsing of joint-name failed! Node:" << endl << node << endl;
			return 0;
		}
		return 0;
	}

	EffortLimitExpr* 	ExpressionTree::parseEffortLimitExpr(const YAML::Node& node){
		try{
			string jointName = node.as<string>();
			if (sim->hasJoint(jointName)) {
				boost::shared_ptr<EffortLimitExpr> ptr(new EffortLimitExpr(sim->getJoint(jointName)->limits));
				doubleExpressions.push_back(ptr);
				return ptr.get();
			} else
			cerr << "Unknown joint '" << jointName << "' while parsing EffortLimit expression! Node:" << endl << node << endl;

		} catch (const YAML::Exception& e) {
			cerr << "Parsing of joint-name failed! Node:" << endl << node << endl;
			return 0;
		}
		return 0;
	}

}