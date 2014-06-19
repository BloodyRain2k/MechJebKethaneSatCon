/*
 * Created by SharpDevelop.
 * User: Bernhard
 * Date: 16.07.2013
 * Time: 21:02
 */

using System;
using System.Collections.Generic;
using System.Linq;
//using System.Text;
//using System.Reflection;
using Kethane;
using UnityEngine;

namespace MuMech
{
	public class MechJebModuleKethaneSatelliteController : ComputerModule
	{
		private Cell[] ordered = Cell.AtLevel(5).OrderByDescending(c => Math.Abs(c.Position.y)).ToArray();
		public DateTime LastScan;
		public float maxInc;
		public ControllerState Status;
		public float Coverage;
//		private CelestialBody lastBody;
		public List<float> Orbits;
		public string selectedResource;
//		static FieldInfo fiSelRes;
		
		public enum ControllerState
		{
			Inactive, Idle, Waiting, Disabled
		}
		
		// usable scanning orbitperiods: orbitP = (160 * sideP / 1-160)

		public bool Active = false;
		[Persistent(pass = (int)(Pass.Type | Pass.Global))]
		public bool ChangeByStep = false;
		[Persistent(pass = (int)(Pass.Type | Pass.Global))]
		public EditableDouble IncChangeStep = new MuMech.EditableDouble(1.5);
		[Persistent(pass = (int)(Pass.Type | Pass.Global))]
		public EditableDouble IncChangeLimit = new MuMech.EditableDouble(15);
		public int warp;

		public MechJebModuleKethaneSatelliteController(MechJebCore core) : base(core) { }
		
		public Cell? GetLowestUnscanned(Cell.Set scans)
		{
			LastScan = DateTime.Now;
			
			foreach (var cell in ordered)
			{
				if (!scans[cell]) { return cell; }
			}
			return null;
		}
		
		public Vector2 CellToLonLat(Cell? cell)
		{
			if (cell == null) { return Vector2.zero; }
			var pos = cell.Value.Position;
			var lat = (float)(Math.Atan2(pos.y, Math.Sqrt(pos.x * pos.x + pos.z * pos.z)) * 180 / Math.PI);
			var lon = (float)(Math.Atan2(pos.z, pos.x) * 180 / Math.PI);
			return new Vector2(lon, lat);
		}
		
//		public override void OnStart(PartModule.StartState state)
//		{
//			base.OnStart(state);
//			
//			fiSelRes = Type.GetType("Kethane.KethaneController").GetField("SelectedResource");
//		}
		
		public override void OnFixedUpdate()
		{
//			base.OnFixedUpdate();
			if (HighLogic.LoadedScene != GameScenes.FLIGHT || KethaneData.Current == null) { return; }
			
//			if (lastBody != vessel.orbit.referenceBody) {
//				lastBody = vessel.orbit.referenceBody;
//				Orbits = new List<float>();
//				float y = 1;
//				while (y > 0)
//				{
//					var oP = 160f * (float)lastBody.rotationPeriod / y;
//					var u3 = Math.Pow(lastBody.gravParameter, 1.0 / 3.0);
//					float a = (float)((Math.Pow(oP, 2.0 / 3.0) * u3) / Math.Pow(Math.PI, 2.0 / 3.0));
//					a -= (float)lastBody.Radius;
//					var msg = string.Format("Orbit {0} is at {1:F3} km", y, a / 1000f);
//					if (a > 0 && a < lastBody.sphereOfInfluence && (lastBody.atmosphere ? a > lastBody.RealMaxAtmosphereAltitude() : true))
//					{
//						msg += " added";
//						Orbits.Add(a / 1000f);
//					}
////					Debug.Log(msg);
//					y++;
//					if (a <= 0) { break; }
//				}
//				Orbits.Sort();
//				string str = "";
//				foreach (var o in Orbits)
//				{
//					str += o.ToString("#.000") + " km\n";
//				}
//				System.IO.File.WriteAllText(KSPUtil.ApplicationRootPath + "\\KethaneScanOrbits_" + lastBody.name + ".txt", str);
//			}	
			
//			var scans = KethaneData.Current.Scans[(string)fiSelRes.GetValue(null)][vessel.mainBody.name];
			var scans = KethaneData.Current.Scans[MapOverlay.SelectedResource][vessel.mainBody.name];
			Coverage = (float)Math.Floor(ordered.Count(c => scans[c]) / (float)ordered.Length * 100f * 100f) / 100f;
			
			var cell = GetLowestUnscanned(scans);
			if (Coverage == 100 && Active) {
				print(vessel.mainBody.name + " completely scanned at " + vesselState.time);
				TimeWarp.SetRate(0, false);
				Active = false;
			}
			maxInc = CellToLonLat(cell).y;
			
			if (Active)
			{
				var curInc = Math.Abs(maxInc);
				var myInc = Math.Abs(vesselState.orbitInclination);
				var diff = Math.Abs(myInc - curInc);

				switch (Status)
				{
					case ControllerState.Inactive:
						Status = ControllerState.Idle;
						break;
						
					case ControllerState.Idle:
						if (diff > IncChangeStep && myInc - IncChangeStep >= IncChangeLimit && vessel.patchedConicSolver.maneuverNodes.Count == 0) {
							var planner = core.GetComputerModule<MechJebModuleManeuverPlanner>();
							
							double newInc;
							if (ChangeByStep) {
								newInc = vesselState.orbitInclination - (IncChangeStep * (vesselState.orbitInclination < 0 ? -1 : 1));
							}
							else
							{
								newInc = (curInc * (vesselState.orbitInclination < 0 ? -1 : 1));
							}
							newInc = Math.Round(newInc, 2);
							
							var result = planner.PlanNode(MechJebModuleManeuverPlanner.Operation.INCLINATION,
							                              MechJebModuleManeuverPlanner.TimeReference.EQ_ASCENDING, 0, 0, 0, newInc, 0, 0, 0, false);

							if (result.Success && core.node != null) {
								core.node.ExecuteOneNode(this);
							}
							else {
								Active = false;
								Status = ControllerState.Inactive;
								print("KSC node planning error: " + result.Error + "\ntime error: " + result.TimeError);
							}

							warp = TimeWarp.CurrentRateIndex;
							TimeWarp.SetRate(0, true);
							Status = ControllerState.Waiting;
						}
						break;
						
					case ControllerState.Waiting:
						if (vessel.patchedConicSolver.maneuverNodes.Count == 0 && vessel.ctrlState.mainThrottle == 0)
						{
							TimeWarp.SetRate(warp, false);
							Status = ControllerState.Idle;
//							if (myInc - IncChangeStep < IncChangeLimit) { Active = false; }
						}
						break;
				}
				
//				if (!core.node.users.Contains(this)) { core.node.users.Add(this); }
			}
			else //if(!Active)
			{
				if (Status != ControllerState.Disabled && Status != ControllerState.Inactive) { Status = ControllerState.Inactive; }
//				if (core.node.users.Contains(this)) { core.node.users.Remove(this); }
			}
		}
	}
	
	
	
	public class MechJebModuleKethaneSatelliteControllerWindow : DisplayModule
	{
		public MechJebModuleKethaneSatelliteControllerWindow(MechJebCore core) : base(core) { }

		public MechJebModuleKethaneSatelliteController controller;
		
		public override void OnStart(PartModule.StartState state)
		{
			controller = core.GetComputerModule<MechJebModuleKethaneSatelliteController>();
			controller.enabled = true;
		}

		public override string GetName()
		{
			return "Kethane Satellite Controller";
		}

		public override GUILayoutOption[] WindowOptions()
		{
			return new GUILayoutOption[] { GUILayout.Width(200), GUILayout.Height(50) };
		}

		protected override void WindowGUI(int windowID)
		{
			GUILayout.BeginVertical();
			
			GUILayout.BeginHorizontal();
			controller.Active = GUILayout.Toggle(controller.Active, "Active" + (controller.Active ? " - Status:" : ""), GUILayout.ExpandWidth(true));
			GUILayout.Label((controller.Active ? controller.Status.ToString() : " "), GUILayout.ExpandWidth(false));
			GUILayout.EndHorizontal();

			controller.ChangeByStep = GUILayout.Toggle(controller.ChangeByStep, "Change by Step");
			
			GUILayout.BeginHorizontal();
			GUILayout.Label("Change Step", GUILayout.ExpandWidth(true));
			controller.IncChangeStep.text = GUILayout.TextField(controller.IncChangeStep.text, GUILayout.Width(35), GUILayout.ExpandWidth(false));
			GUILayout.Label("°", GUILayout.ExpandWidth(false));
			GUILayout.EndHorizontal();

			GUILayout.BeginHorizontal();
			GUILayout.Label("Change Limit", GUILayout.ExpandWidth(true));
			controller.IncChangeLimit.text = GUILayout.TextField(controller.IncChangeLimit.text, GUILayout.Width(35), GUILayout.ExpandWidth(false));
			GUILayout.Label("°", GUILayout.ExpandWidth(false));
			GUILayout.EndHorizontal();
			
			GUILayout.BeginHorizontal();
			GUILayout.Label("Coverage", GUILayout.ExpandWidth(true));
			GUILayout.Label(controller.Coverage.ToString("F2") + " %", GUILayout.ExpandWidth(false));
			GUILayout.EndHorizontal();
			
			GUILayout.BeginHorizontal();
			GUILayout.Label("max. Inc °", GUILayout.ExpandWidth(true));
			GUILayout.Label(controller.maxInc.ToString("F1"), GUILayout.ExpandWidth(false));
			GUILayout.EndHorizontal();
			
//			GUILayout.Label("Suggested Orbits");
//			int a = controller.Orbits.FindIndex(o => o > vessel.altitude);
//			if (a - 3 >= 0) {
//				for (var i = a - 3; i < controller.Orbits.Count; i++)
//				{
//					GUILayout.Label(string.Format("{0:F3} km", controller.Orbits[i]));
//					if (i >= 2) { break; }
//				}
//			}

//			MechJebModuleCustomWindowEditor ed = core.GetComputerModule<MechJebModuleCustomWindowEditor>();
//			ed.registry.Find(i => i.id == "Toggle:RoverController.ControlHeading").DrawItem();
//			ed.registry.Find(i => i.id == "Editable:RoverController.heading").DrawItem();
//			ed.registry.Find(i => i.id == "Value:RoverController.headingErr").DrawItem();
//			ed.registry.Find(i => i.id == "Toggle:RoverController.ControlSpeed").DrawItem();
//			ed.registry.Find(i => i.id == "Editable:RoverController.speed").DrawItem();
//			ed.registry.Find(i => i.id == "Value:RoverController.speedErr").DrawItem();

			GUILayout.EndVertical();
			
			base.WindowGUI(windowID);
		}
	}
}