package sim.app.exploration.agents;

import java.util.*;
import java.util.concurrent.Phaser;
import java.util.concurrent.atomic.AtomicInteger;

import com.lowagie.text.pdf.AcroFields;
import sim.app.exploration.env.SimEnvironment;
import sim.app.exploration.objects.*;
import sim.app.exploration.utils.Utils;
import sim.engine.SimState;
import sim.util.Bag;
import sim.util.Double2D;
import sim.util.Int2D;

import javax.swing.*;


public class ExplorerAgent implements sim.portrayal.Oriented2D {

	public static final int HIGH= 0;
	public static final int LOW = 1;
	public static final int EMPTY = 2;
	public static final int FULL = 3;


	public static final int CHARGE = 1;
	public static final int TARGET = 2;
	private static final long serialVersionUID = 1L;
	private float INTEREST_THRESHOLD = 65;
	private final double STEP = Math.sqrt(2);
	private final int viewRange = 40;
	private final int ENERGY = 90;


	private final Hashtable<Integer, Set<Class>> observation = new Hashtable<Integer,Set<Class>>();


	private int identifyClock;

	private Int2D loc;
	private Int2D target;
	private int priority;
	private double orientation;
    private int leftEnergy;
    private int startEnergy;
    private int energyState;
	public SimEnvironment env;
	public BrokerAgent broker;
	public MapperAgent mapper;
	private Vector<Prototype> knownObjects;
	private double energyNeededToNearestChargingToTarget;
	private double energyToNearestCharging;
	private double energyNeededToTarget;
	private boolean GLOBAL_KNOWLEDGE = true;
	private int IDENTIFY_TIME = 15;
	private HashMap<Class,Integer> timeWithoutInspecting;
	public ExplorerAgent(Int2D loc) {
		this.loc = loc;
		this.orientation = 0;
		this.target = null;
		this.knownObjects = new Vector<Prototype>();
		this.identifyClock = 0;
		this.timeWithoutInspecting = new HashMap<Class,Integer>();
		this.leftEnergy = ENERGY;
		this.energyState = HIGH;
		this.priority = TARGET;
	}

	public void step(SimState state) {

		// The explorer sees the neighboring objects and sends them to the
		// mapper

		if (identifyClock == 0) {
			Bag visible = env.getVisibleObejcts(loc.x, loc.y, viewRange);

			// -------------------------------------------------------------
			for (int i = 1; i < visible.size(); i++) {
				SimObject obj = (SimObject) visible.get(i);

				if (!mapper.isIdentified(obj.loc)) {
					Hashtable<Class, Double> probs = getProbabilityDist(obj);



					float interest = getObjectInterest(probs);
					/*System.out.println("OBJECT AT: (" + obj.loc.x + ","
							+ obj.loc.y + "). INTEREST: " + interest + obj.getClass());*/

					// If not interesting enough, classify it to the highest prob
					if (interest < INTEREST_THRESHOLD) {
						Class highest = Utils.getHighestProb(probs);

						mapper.identify(obj, highest);
						Class real = env.identifyObject(obj.loc).getClass();
						//if (highest != real)
						//	System.err.println(real.getSimpleName());
						
						broker.removePointOfInterest(obj.loc);
					} else {
						mapper.addObject(obj);
						broker.addPointOfInterest(obj.loc, interest);

					}
				}

			}
			// --------------------------------------------------------------

			// Check to see if the explorer has reached its target

			if (target != null && priority == TARGET) {

				if (loc.distance(target) == 0) {
					target = null;

					SimObject obj = env.identifyObject(loc);
					System.out.println("Energy Expended->"+(startEnergy-leftEnergy));
					leftEnergy = 90;
					if (obj != null) {
						broker.removePointOfInterest(obj.loc);
						mapper.identify(obj, obj.getClass());
						addPrototype(obj, obj.getClass());

						timeWithoutInspecting.put(obj.getClass(),0);
						identifyClock = IDENTIFY_TIME;
					}
				}
			}else if(target != null && priority == CHARGE){
				if(loc.distance(mapper.getNearestChargingStation(loc))==0){
					recharge(30);
					if (energyState == HIGH) {
						priority = TARGET;
					}
				}
			}

			// If the explorer has no target, he has to request a new one from
			// the broker
			if (target == null) {
				target = broker.requestTarget(loc);
				startEnergy = leftEnergy;
				energyNeededToNearestChargingToTarget = 0;
				energyToNearestCharging = 0;

				double energyNeededToTarget  = loc.distance(target)/2;
				if (mapper.getNearestChargingStation(target)!= null & mapper.getNearestChargingStation(loc)!= null) {
					energyNeededToNearestChargingToTarget = target.distance(mapper.getNearestChargingStation(target)) / 2;
					energyToNearestCharging = loc.distance(mapper.getNearestChargingStation(loc))/2;

					System.out.println("Target distance : "+ loc.distance(target)+"-t energy needed :" +energyNeededToTarget +"\nNearest Charging Station :" + mapper.getNearestChargingStation(loc) +"-t energy required" + energyToNearestCharging +"\nTotal " +target.distance(mapper.getNearestChargingStation(target)) +Math.round(energyNeededToNearestChargingToTarget));

					if((energyNeededToTarget+energyNeededToNearestChargingToTarget)<leftEnergy)
						priority = CHARGE;
					else{
						if(energyNeededToTarget<=energyToNearestCharging){
							priority = TARGET;
						}
						else{
							priority = CHARGE;
						}
					}

				}




				//System.out.println("NEW TARGET: X: " + target.x + " Y: "

				//		+ target.y);
			}

			// Agent movement
			Double2D step;
			//MOVETOSTATION
			if(energyState == EMPTY && leftEnergy<45){//DONT MOVE SLOW RECHARGE UNTIL HALF ENERGY
				step = new Double2D(0, 0);
				recharge(50);
			}
			else if (priority == CHARGE){
				step = new Double2D(mapper.getNearestChargingStation(loc).x - loc.x, mapper.getNearestChargingStation(loc).y - loc.y);
			}
			else{
				step = new Double2D(target.x - loc.x, target.y - loc.y);
			}
			//MOVETOTARGET

			step.limit(STEP);

			loc.x += Math.round(step.x);
			loc.y += Math.round(step.y);
			System.out.println("Distance to Target->"+loc.distance(target));

			env.updateLocation(this, loc);
			mapper.updateLocation(this, loc);

			orientation = Math.atan2(Math.round(step.y), Math.round(step.x));

		}
		
		if (identifyClock > 0)
			identifyClock--;
	}

	private int getObjectInterest(Hashtable<Class, Double> probs) {
		double unknownInterest = 0;
		double entropyInterest;
		int hunger;
		Vector<Double> prob = new Vector<Double>();

		for (Class c : probs.keySet()) {
			if (c == SimObject.class)
				unknownInterest = Utils.interestFunction(probs.get(c));

			prob.add(probs.get(c));
		}

		/*for (Map.Entry<Class,Double> entry : probs.entrySet()) {
			String key = entry.getKey().toString();
			Double value = entry.getValue();

			System.out.println ("Key: " + key + " Value: " + value);
		}
		System.out.println(prob.toString());*/
		entropyInterest = Utils.entropy(prob);
/*
		System.out.println("ENTROPY: " + entropyInterest + " | UNKNOWN: "
				+ unknownInterest);*/


		hunger = this.hunger(leftEnergy);
		double interest = (entropyInterest > unknownInterest ? entropyInterest : unknownInterest) * 100;

		return (int) Math.round(interest);
	}

	private void addPrototype(SimObject obj, Class class1) {
		// TODO Auto-generated method stub

		// Using the global team knowledge
		if (GLOBAL_KNOWLEDGE) {

			mapper.addPrototype(obj, class1);

			// Using only the agent's knowledge
		} else {
			for (Prototype p : this.knownObjects) {
				if (class1 == p.thisClass) {
					p.addOccurrence(obj.size, obj.color);
					return;
				}
			}

			this.knownObjects.add(new Prototype(class1, obj.size, obj.color));
		}

		this.timeWithoutInspecting.put(class1,0);

	}
	private void recharge(int ammount){
		leftEnergy+=ammount;
		if(leftEnergy>45)
			energyState = HIGH;

	}
	private Hashtable<Class, Double> getProbabilityDist(SimObject obj) {

		Hashtable<Class, Double> probs = new Hashtable<Class, Double>();

		// TODO: Implement global knowledge

		Vector<Prototype> prototypes;
		if (GLOBAL_KNOWLEDGE) {
			prototypes = mapper.knownObjects;
		} else {
			prototypes = this.knownObjects;
		}
		int nClasses = prototypes.size();
		double unknownCorr = 0;
		double corrSum = 0;

		for (Prototype prot : prototypes) {
			// TODO: Stuff here
			double corr;
			double colorDist = Utils.colorDistance(obj.color, prot.color);
			double sizeDist = Math.abs(obj.size - prot.size) / Utils.MAX_SIZE;

			// Correlation
			corr = 1 - (0.5 * colorDist + 0.5 * sizeDist);
			// Saturation
			corr = Utils.saturate(corr, prot.nOccurrs);

			probs.put(prot.thisClass, corr*corr*corr);
			corrSum += corr*corr*corr;

			unknownCorr += (1 - corr) / nClasses;
		}

		if (nClasses == 0)
			unknownCorr = 1.0;
		probs.put(SimObject.class, unknownCorr*unknownCorr*unknownCorr);
		corrSum += unknownCorr*unknownCorr*unknownCorr;

		for (Class c : probs.keySet()) {
			
			probs.put(c, probs.get(c) / corrSum);
			//System.out.println(c.getSimpleName() + " : " + probs.get(c));
		}

		return probs;
	}


	private int hunger(int energy) {
	    int hunger = ENERGY;
	    return -(hunger - energy);
    }
    public void expendEnergy(){
		this.leftEnergy--;
		if(leftEnergy<30) {
			energyState = LOW;
		}
	}

	@Override
	public double orientation2D() {
		return orientation;
	}

	public Int2D getLoc() {
		return loc;
	}

	public double getOrientation() {
		return orientation;
	}


}
