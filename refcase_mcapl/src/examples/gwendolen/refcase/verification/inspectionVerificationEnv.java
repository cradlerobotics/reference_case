package gwendolen.refcase.verification;

import java.util.HashSet;
import java.util.Set;
import ail.syntax.*;

import ail.syntax.Literal;
import ail.syntax.Message;
import ail.syntax.Predicate;
import ail.syntax.VarTerm;
import gwendolen.mas.VerificationofAutonomousSystemsEnvironment;


public class inspectionVerificationEnv extends VerificationofAutonomousSystemsEnvironment {

    @Override
    public Set<Predicate> generate_percepts() {
        Set<Predicate> beliefs = new HashSet<Predicate>();


        boolean geofenceViolation = random_bool_generator.nextBoolean();
        boolean safeHaltReq = random_bool_generator.nextBoolean();
        boolean robotStopped = safeHaltReq && random_bool_generator.nextBoolean();



        if (geofenceViolation) {
            beliefs.add(new Predicate("geofence_violation"));
            System.out.println("adding geofence_violation");
        }

        if (safeHaltReq){
            beliefs.add(new Predicate("safe_halt_req"));
            System.out.println("adding safe_halt_req");

        }

        if (robotStopped){
            beliefs.add(new Predicate("halt_observed"));
            System.out.println("adding halt_observed");
        } else {
            beliefs.add(new Predicate("move"));
            System.out.println("adding move");
        }

        return beliefs;
    }

    public Unifier executeAction(String agName, Action act) {

        if ("waiting".equals(act.getFunctor())) {
            addPercept(new Literal("tick"));
            System.out.println("TICK GENERATED");
        }

        return new Unifier();
    }

    @Override
    public Set<Message> generate_messages() {
        return new HashSet<Message>();
    }
}

