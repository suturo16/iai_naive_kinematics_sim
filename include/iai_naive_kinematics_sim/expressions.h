#pragma once
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <yaml-cpp/yaml.h>

#include <string>
#include <unordered_map>

using namespace std;

namespace iai_naive_kinematics_sim
{
template <typename A>
	struct Expression {
		virtual A value() = 0;
	};

template <typename A, typename B>
	struct UnaryExpression : public Expression<A> {
		UnaryExpression(B* _arg) : arg(_arg) {}

	protected:
		B* arg;
	};

template <typename A, typename B, typename C>
	struct BinaryExpression : public Expression<A> {
		BinaryExpression(B* _r, C* _l) : right(_r), left(_l) {}

	protected:
		B* right;
		C* left;
	};

template <typename A>
	struct JointExprBase : public Expression<A> {
		JointExprBase(sensor_msgs::JointState& _state) : state(_state) {}

	protected:
		sensor_msgs::JointState& state;
	};

template <typename A>
	struct UnaryJointExpr : public JointExprBase<A> {
		UnaryJointExpr(sensor_msgs::JointState& _state, size_t _idx) : JointExprBase<A>(_state), idx(_idx) {}
	protected:
		size_t idx;
	};

// ----------- CONCRETE EXPRESSIONS --------------

	struct ConstDoubleExpr : public Expression<double> {
		ConstDoubleExpr(double a) : v(a) {}

		inline double value() { return v; }
	private:
		double v;
	};

// ----------- MATH STUFF ------------------------

	struct AddExpr : public BinaryExpression<double, Expression<double>, Expression<double>> {
		AddExpr(Expression<double>* a, Expression<double>* b) : BinaryExpression<double, Expression<double>, Expression<double>>(a, b) {}
		inline double value() { return right->value() + left->value(); }
	};

	struct SubExpr : public BinaryExpression<double, Expression<double>, Expression<double>> {
		SubExpr(Expression<double>* a, Expression<double>* b) : BinaryExpression<double, Expression<double>, Expression<double>>(a, b) {}
		inline double value() { return right->value() - left->value(); }
	};

	struct MulExpr : public BinaryExpression<double, Expression<double>, Expression<double>> {
		MulExpr(Expression<double>* a, Expression<double>* b) : BinaryExpression<double, Expression<double>, Expression<double>>(a, b) {}
		inline double value() { return right->value() * left->value(); }
	};

	struct DivExpr : public BinaryExpression<double, Expression<double>, Expression<double>> {
		DivExpr(Expression<double>* a, Expression<double>* b) : BinaryExpression<double, Expression<double>, Expression<double>>(a, b) {}
		inline double value() { return right->value() / left->value(); }
	};

	struct MinExpr : public BinaryExpression<double, Expression<double>, Expression<double>> {
		MinExpr(Expression<double>* a, Expression<double>* b) : BinaryExpression<double, Expression<double>, Expression<double>>(a, b) {}
		inline double value() { return min(right->value(), left->value()); }
	};

	struct MaxExpr : public BinaryExpression<double, Expression<double>, Expression<double>> {
		MaxExpr(Expression<double>* a, Expression<double>* b) : BinaryExpression<double, Expression<double>, Expression<double>>(a, b) {}
		inline double value() { return max(right->value(), left->value()); }
	};

	struct AbsExpr : public UnaryExpression<double, Expression<double>> {
		AbsExpr(Expression<double>* a) : UnaryExpression<double, Expression<double>>(a) {}
		inline double value() { return abs(arg->value()); }
	};

	struct SinExpr : public UnaryExpression<double, Expression<double>> {
		SinExpr(Expression<double>* a) : UnaryExpression<double, Expression<double>>(a) {}
		inline double value() { return sin(arg->value()); }
	};

	struct CosExpr : public UnaryExpression<double, Expression<double>> {
		CosExpr(Expression<double>* a) : UnaryExpression<double, Expression<double>>(a) {}
		inline double value() { return cos(arg->value()); }
	};

// ------------ JOINT STUFF ----------------
	typedef boost::shared_ptr<urdf::JointLimits> LimitPtr;

	struct JointLimitContainer {
		JointLimitContainer(LimitPtr ptr) : limits(ptr) {}

	protected:
		LimitPtr limits;
	};

	struct PositionExpr : public UnaryJointExpr<double> {
		PositionExpr(sensor_msgs::JointState &state, size_t _idx)
		: UnaryJointExpr<double>(state, _idx) {}
		inline double value() {return state.position[idx]; }
	};

	struct VelocityExpr : public UnaryJointExpr<double> {
		VelocityExpr(sensor_msgs::JointState &state, size_t _idx)
		: UnaryJointExpr<double>(state, _idx) {}
		inline double value() {return state.velocity[idx]; }
	};

	struct EffortExpr : public UnaryJointExpr<double> {
		EffortExpr(sensor_msgs::JointState &state, size_t _idx)
		: UnaryJointExpr<double>(state, _idx) {}
		inline double value() {return state.effort[idx]; }
	};

	struct PositionFracExpr : public UnaryJointExpr<double>, JointLimitContainer {
		PositionFracExpr(sensor_msgs::JointState &state, size_t _idx, LimitPtr ptr)
		: UnaryJointExpr<double>(state, _idx), JointLimitContainer(ptr) {}
		inline double value() { return (state.position[idx] - limits->lower) / (limits->upper - limits->lower); }
	};

	struct VelocityFracExpr : public UnaryJointExpr<double>, JointLimitContainer {
		VelocityFracExpr(sensor_msgs::JointState &state, size_t _idx, LimitPtr ptr)
		: UnaryJointExpr<double>(state, _idx), JointLimitContainer(ptr) {}
		inline double value() { return state.velocity[idx] / limits->velocity; }
	};

	struct EffortFracExpr : public UnaryJointExpr<double>, JointLimitContainer {
		EffortFracExpr(sensor_msgs::JointState &state, size_t _idx, LimitPtr ptr)
		: UnaryJointExpr<double>(state, _idx), JointLimitContainer(ptr) {}
		inline double value() { return state.effort[idx] / limits->effort; }
	};

	struct PosUpLimitExpr : public Expression<double>, JointLimitContainer {
		PosUpLimitExpr(LimitPtr ptr)
		: JointLimitContainer(ptr) {}
		inline double value() { return limits->upper; }
	};

	struct PosLowLimitExpr : public Expression<double>, JointLimitContainer {
		PosLowLimitExpr(LimitPtr ptr)
		: JointLimitContainer(ptr) {}
		inline double value() { return limits->lower; }
	};

	struct PosLimitSpreadExpr : public Expression<double>, JointLimitContainer {
		PosLimitSpreadExpr(LimitPtr ptr)
		: JointLimitContainer(ptr) {}
		inline double value() { return limits->upper - limits->lower; }
	};

	struct VelocityLimitExpr : public Expression<double>, JointLimitContainer {
		VelocityLimitExpr(LimitPtr ptr)
		: JointLimitContainer(ptr) {}
		inline double value() { return limits->velocity; }
	};

	struct EffortLimitExpr : public Expression<double>, JointLimitContainer {
		EffortLimitExpr(LimitPtr ptr)
		: JointLimitContainer(ptr) {}
		inline double value() { return limits->effort; }
	};

	class Simulator;

	class ExpressionTree {
	public:
		ExpressionTree(Simulator* simulator) : sim(simulator) {}

		bool parseYAML(const YAML::Node& node,
					   unordered_map<size_t, Expression<double>*>& posExprs,
					   unordered_map<size_t, Expression<double>*>& velExprs,
					   unordered_map<size_t, Expression<double>*>& effExprs);
	private:
		Expression<double>* parseDoubleExpr(const YAML::Node& node);
		AddExpr* parseAddExpr(const YAML::Node& node);
		SubExpr* parseSubExpr(const YAML::Node& node);
		MulExpr* parseMulExpr(const YAML::Node& node);
		DivExpr* parseDivExpr(const YAML::Node& node);
		MinExpr* parseMinExpr(const YAML::Node& node);
		MaxExpr* parseMaxExpr(const YAML::Node& node);
		AbsExpr* parseAbsExpr(const YAML::Node& node);
		SinExpr* parseSinExpr(const YAML::Node& node);
		CosExpr* parseCosExpr(const YAML::Node& node);

		PositionExpr* 	parsePositionExpr(const YAML::Node& node);
		VelocityExpr* 	parseVelocityExpr(const YAML::Node& node);
		EffortExpr* 	parseEffortExpr(const YAML::Node& node);

		PositionFracExpr* 	parsePositionFracExpr(const YAML::Node& node);
		VelocityFracExpr* 	parseVelocityFracExpr(const YAML::Node& node);
		EffortFracExpr* 	parseEffortFracExpr(const YAML::Node& node);

		PosLowLimitExpr* 	parsePosLowLimitExpr(const YAML::Node& node);
		PosUpLimitExpr* 	parsePosUpLimitExpr(const YAML::Node& node);
		PosLimitSpreadExpr* parsePosLimitSpreadExpr(const YAML::Node& node);

		VelocityLimitExpr* 	parseVelocityLimitExpr(const YAML::Node& node);
		EffortLimitExpr* 	parseEffortLimitExpr(const YAML::Node& node);

		Simulator* sim;

		unordered_map<string, Expression<double>*> namedDoubleExpr;
		vector<boost::shared_ptr<Expression<double>>> doubleExpressions;
	};
}