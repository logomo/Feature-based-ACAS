%% Development of the rule engine setup
obj = RuleEngine();
% add some rule codes to joint point
obj.activateRules(RuleJointPoint.MissionControlRunOnce,[...
    RuleCode.PriorRulesOfAir,...
    RuleCode.PostRulesOfAir,...
    RuleCode.ConvergingManeuver,...
    RuleCode.HeadOnApproachManeuver,...
    RuleCode.OvertakeManevuer]);
% remove currently not implemented rules (no invocation)
obj.deactivateRules(RuleJointPoint.MissionControlRunOnce,[...
    RuleCode.HeadOnApproachManeuver,...
    RuleCode.OvertakeManevuer]);
% get active rule codes
activeRules=obj.getRuleCodes(RuleJointPoint.MissionControlRunOnce);
%create abstract rule;
context=containers.Map;
context('ruleEngine')= obj;             % default context rule engine for additional rule invocation
context('utmControl')=UTMControl;       % default context utm as highest level instance
ar=AbstractRule(context,RuleJointPoint.MissionControlRunOnce,RuleCode.ConvergingManeuver);
ar.runRule;
%test rule
tr=TestRule(context,RuleJointPoint.MissionControlRunOnce,RuleCode.ConvergingManeuver);
tr.runRule;
