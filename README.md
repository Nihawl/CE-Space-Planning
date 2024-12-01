//Exhibition Hall Space Planning Automation
//C# Definition Code


//Component a
//Algorithm
    //Definition of Main Exhibtion Area
    Plane MainPlane = Plane.WorldXY;
    Vector3d VecNorm = new Vector3d(0.0, 0.0, 1.0);
    Rectangle3d MainRec = new Rectangle3d(MainPlane, W, D);

    //Back Edge Booths
    Point3d E_pt0 = new Point3d(OPoint.X + Offset_Value, OPoint.Y + D - 0.2, OPoint.Z);
    Point3d E_pt1 = new Point3d(OPoint.X + (W - Offset_Value), OPoint.Y + D - 0.2 - BorderBoothW, OPoint.Z);
    Rectangle3d BackRec = new Rectangle3d(MainPlane, E_pt0, E_pt1);

    //Side Walls Booths
    List<Rectangle3d> SRec = new List<Rectangle3d>();
    //Left Edge
    //Create Left Edge List of Points
    List<Point3d> LEPt = new List<Point3d> ();
    //Set start and end points
    Point3d LEStartPt = new Point3d (OPoint.X + 0.12, OPoint.Y + Entrance_Area_Depth, OPoint.Z);
    Point3d LEEndPt = new Point3d (OPoint.X + 0.12, OPoint.Y + D - (Offset_Value * 0.5), OPoint.Z);
    LEPt.Add(LEStartPt);
    foreach (Point3d pt in Sec_Ent_Pts)
    {
      if (pt.X == 0)
      {
        Point3d pt0 = new Point3d(pt.X + 0.12, pt.Y + (Offset_Value / 2), pt.Z);
        LEPt.Add(pt0);
      }
    }
    LEPt.Add(LEEndPt);
    List<Rectangle3d> LERec = new List<Rectangle3d >();
    List<double> LEDis = new List<double>();
    for (int i = 0; i < LEPt.Count - 1; i++)
    {
      Plane LeftPlane = new Plane(LEPt[i], VecNorm);
      //Calculate Height
      double ledis = LEPt[i].DistanceTo(LEPt[i + 1]) - (Offset_Value );
      LEDis.Add(ledis);
      Rectangle3d leRec = new Rectangle3d(LeftPlane, BorderBoothW, LEDis[i]);
      SRec.Add(leRec);
    }

    //Right Edge
    List<Point3d> REPt = new List<Point3d>();
    //Set start and end points
    Point3d REStartPt = new Point3d(OPoint.X + W - 0.12, OPoint.Y + Entrance_Area_Depth, OPoint.Z);
    Point3d REEndPt = new Point3d(OPoint.X + W - 0.12, OPoint.Y + D - (Offset_Value * 0.5), OPoint.Z);
    REPt.Add(REStartPt);
    foreach (Point3d pt in Sec_Ent_Pts)
    {
      if (pt.X > 0)
      {
        Point3d pt0 = new Point3d(pt.X - 0.12, pt.Y + (Offset_Value / 2), pt.Z);
        REPt.Add(pt0);
      }
    }
    REPt.Add(REEndPt);
    List<Rectangle3d> RERec = new List<Rectangle3d>();
    List<double> REDis = new List<double>();
    for (int i = 0; i < REPt.Count - 1; i++)
    {
      Plane ReftPlane = new Plane(REPt[i], VecNorm);
      //Calculate Height
      double redis = REPt[i].DistanceTo(REPt[i + 1]) - (Offset_Value );
      REDis.Add(redis);
      Rectangle3d reRec = new Rectangle3d(ReftPlane, -BorderBoothW, REDis[i]);
      SRec.Add(reRec);
    }

    //Entrance Edge


    //Output
    Main_Space_Boundary = MainRec;
    Exhibition_SideEdges_Boundary = SRec;
    Exhibition_BackEdges_Boundary = BackRec;
    
//Component b
//Algorithm
    int n = NumOfEdgeBooths / 3;
    Vector3d NormVec = new Vector3d(0.0, 0.0, 1.0);

    //BackEdge Subdivision
    List<Point3d> pts = new List<Point3d>();
    List<Point3d> Enpts = new List<Point3d>();
    List<Rectangle3d> booths = new List<Rectangle3d>();
    Point3d BStP0 = BackEdgeBooth.PointAt(0.0);
    Point3d BEnP0 = BackEdgeBooth.PointAt(1.0);

    double EHd = BStP0.DistanceTo(BEnP0);
    double Et = 1.00 / n;
    double EStep = EHd / n;
    for (int i = 0; i < n; i++)
    {
      Point3d pt = new Point3d(BStP0.X + (EStep * i), BStP0.Y, 0.0);
      Point3d ept = new Point3d(BStP0.X + (EStep * i) + (EStep * 0.5), BStP0.Y - 0.4, 0.0);
      Plane pp = new Plane(pt, NormVec);
      Rectangle3d rec = new Rectangle3d(pp, EStep, BorderBoothW);
      pts.Add(pt);
      Enpts.Add(ept);
      booths.Add(rec);
    }
    //////////////////////////////////////////////////////////////////////////
    //Edge Booths Subdivision
    //Calculate total edge length
    DataTree<Point3d> TEPts = new DataTree<Point3d> ();
    DataTree<Point3d> TMidPts = new DataTree<Point3d> ();
    DataTree<Rectangle3d> TERec = new DataTree<Rectangle3d> ();
    DataTree<Rectangle3d> TERecList = new DataTree<Rectangle3d> ();
    DataTree<double> TVlDis = new DataTree<double> ();

    int pathindex = 0;
    double TotalVlD = 0;
    double vld = 0;
    List <int> NumList = new List <int> ();
    List<Point3d> PtsList = new List<Point3d>();
    List <Rectangle3d> RecList = new List <Rectangle3d>();
    List <double> VlDisList = new List <double>();
    List<double> EStepList = new List <double>();

    List<Point3d> MidPts = new List<Point3d>  ();
    List<Point3d> LeftMidPts = new List<Point3d>  ();
    List<Point3d> RightMidPts = new List<Point3d>  ();
    foreach(List<Rectangle3d> r in EdgeBooths.Branches)
    {
      GH_Path path = new GH_Path(pathindex);
      pathindex = pathindex + 1;
      int En = n * 2;

      foreach(Rectangle3d rr in r)
      {
        Point3d pStart0 = rr.PointAt(0.0);
        Point3d pEnd0 = rr.PointAt(3.0);
        PtsList.Add(pStart0);
        PtsList.Add(pEnd0);
        double vldis = pStart0.DistanceTo(pEnd0);
        VlDisList.Add(vldis);

        Point3d mp0 = rr.PointAt(0.0);
        vld = vldis;

      }
      for (int i = 0; i < VlDisList.Count; i++)
      {
        TotalVlD = TotalVlD + VlDisList[i];
      }
      TEPts.AddRange(PtsList, path);
      TVlDis.AddRange(VlDisList, path);

      for (int i = 0; i < VlDisList.Count; i++)
      {
        int NumPerSpc = Convert.ToInt32(En * (VlDisList[i] / TotalVlD));
        NumList.Add(NumPerSpc);
      }

      for (int i = 0; i < VlDisList.Count; i++)
      {
        double EEStep = VlDisList[i] / NumList[i];
        EStepList.Add(EEStep);
        Rectangle3d mr = r[i];
        RecList.Add(mr);
      }

      //Create Points Data Tree
      TERecList.AddRange(RecList, path);
      int pathindex2 = 0;
      foreach(List<Rectangle3d> mrec in TERecList.Branches)
      {
        GH_Path path2 = new GH_Path(pathindex2);
        pathindex2 = pathindex2 + 1;
        int x = 0;
        foreach(Rectangle3d mr in mrec)
        {
          Point3d mp0 = mr.PointAt(0.0);
          double StepSize = EStepList[x];
          for (int i = 0; i < NumList[x]; i++)
          {
            Point3d mpts = new Point3d(mp0.X,
              mp0.Y + (StepSize * i),
              0.0);


            Plane pp = new Plane(mpts, NormVec);
            Rectangle3d MidRec = new Rectangle3d(pp, BorderBoothW, StepSize);
            Point3d mp = MidRec.Center;
            RecList.Add(MidRec);
            MidPts.Add(mp);
          }
          x = x + 1;
          foreach (Point3d pp in MidPts)
          {
            if (pp.X > BorderBoothW)
            {
              Point3d RightPts = new Point3d (pp.X - (BorderBoothW * 0.5) - 0.8, pp.Y, 0.0);
              RightMidPts.Add(RightPts);

            }
            else
            {
              Point3d leftPts = new Point3d (pp.X + (BorderBoothW * 0.5) + 0.8, pp.Y, 0.0);
              LeftMidPts.Add(leftPts);
            }
          }
        }
        TMidPts.AddRange(RightMidPts, path2);
        TMidPts.AddRange(LeftMidPts, path2);
        TERecList.AddRange(RecList, path2);
        TERecList.AddRange(booths);
      }
    }



    //Output
    Side_EntryPts = TMidPts;
    Back_EntryPts = Enpts;
    Booths = TERecList;
    Booth_D = vld;

    //Component c
    //Algorithm
    //Normal Settings
    Vector3d NormVec = new Vector3d(0.0, 0.0, 1.0);
    Plane MainPlane = new Plane(OPoint, NormVec);
    //Main Geometry
    List<Point3d> P0 = new List<Point3d>();
    List<Rectangle3d> R0 = new List<Rectangle3d>();
    List<Point3d> PV = new List<Point3d>();
    List<Rectangle3d> VlB = new List<Rectangle3d>();

    //Only Vl Aisles Case
    if (NumOfVlAisles > 0 &&
      NumOfHzAisles == 0 &&
      MainVlAisleW == 0 &&
      MainHzAisleW == 0)
    {
      //Create Main Rectangle
      Point3d StP0 = new Point3d(OPoint.X + (0.4 + BorderBoothW + (AisleWidth * 0.5)),
        OPoint.Y + EntranceAreaD,
        OPoint.Z);
      Point3d StP2 = new Point3d(OPoint.X + (W - (0.4 + BorderBoothW + ( AisleWidth * 0.5))),
        OPoint.Y + D - (0.4 + BorderBoothW + BackAreaD),
        OPoint.Z);
      P0.Add(StP0);
      P0.Add(StP2);
      Rectangle3d rec = new Rectangle3d(MainPlane, StP0, StP2);
      Point3d StP1 = rec.PointAt(1);
      Point3d StP3 = rec.PointAt(3);
      P0.Add(StP1);
      P0.Add(StP3);
      R0.Add(rec);

      //Create Vl Aisles
      double t = 1 / Convert.ToDouble(NumOfVlAisles);
      double Vd = StP0.DistanceTo(StP3);
      double Hd = StP0.DistanceTo(StP1);
      double BW = (Hd / NumOfVlAisles) - (AisleWidth);

      foreach(Rectangle3d r in R0)
      {
        for (int i = 1; i < NumOfVlAisles + 1; i++)
        {
          Point3d p0 = r.PointAt(t * i);
          Point3d p1 = new Point3d(p0.X - (0.5 * AisleWidth), p0.Y, 0.0);
          Plane pp = new Plane(p1, NormVec);
          Rectangle3d rr = new Rectangle3d(pp, -BW, Vd);
          PV.Add(p0);
          VlB.Add(rr);
        }
      }
    }
    //////////////////////////////////////////////////////////////////////
    //Vl and Hz Aisles Case
    List<Point3d> P1 = new List<Point3d>();

    //    List<Rectangle3d> R1 = new List<Rectangle3d>();
    if (NumOfVlAisles > 0 &&
      NumOfHzAisles > 0 &&
      MainVlAisleW == 0 &&
      MainHzAisleW == 0)
    {
      Point3d StP0 = new Point3d(OPoint.X + 0.12 + BorderBoothW + (AisleWidth * 0.5),
        OPoint.Y + (EntranceAreaD - AisleWidth),
        OPoint.Z);
      Point3d StP1 = new Point3d(OPoint.X + 0.12 + BorderBoothW + (AisleWidth * 0.5),
        OPoint.Y + (D - 0.24 - BorderBoothW - BackAreaD),
        OPoint.Z);
      P1.Add(StP0);
      P1.Add(StP1);
      double dis = StP0.DistanceTo(StP1);
      double Bh = (dis / (NumOfHzAisles + 1)) - AisleWidth;


      for (int i = 0 ; i < NumOfHzAisles + 2 ; i++)
      {
        double xv = (dis / (NumOfHzAisles + 1)) * i;
        Point3d p0 = new Point3d(StP0.X, StP0.Y + (xv + AisleWidth), StP0.Z);
        Plane pl = new Plane(p0, NormVec);
        P0.Add(p0);
      }

      for (int i = 0; i < P0.Count - 1; i++)
      {
        Plane pl = new Plane(P0[i], NormVec);
        Rectangle3d rec = new Rectangle3d(pl,
          W - (BorderBoothW * 2 + AisleWidth + 0.24),
          Bh);
        R0.Add(rec);
      }

      //Create Vl Aisles
      foreach(Rectangle3d r in R0)
      {
        Point3d p0 = r.PointAt(0);
        Point3d p1 = r.PointAt(1);
        Point3d p2 = r.PointAt(2);
        Point3d p3 = r.PointAt(3);
        P0.Add(p1);
        P0.Add(p2);
        P0.Add(p3);

        double t = 1 / Convert.ToDouble(NumOfVlAisles);
        double Vd = p0.DistanceTo(p3);
        double Hd = p0.DistanceTo(p1);
        double BW = (Hd / NumOfVlAisles) - (AisleWidth);
        for (int i = 1; i < NumOfVlAisles + 1; i++)
        {
          Point3d p00 = r.PointAt(t * i);
          Point3d p11 = new Point3d(p00.X - (0.5 * AisleWidth), p00.Y, 0.0);
          Plane pp = new Plane(p11, NormVec);
          Rectangle3d rr = new Rectangle3d(pp, -BW, Vd);
          PV.Add(p00);
          VlB.Add(rr);
        }
      }
    }

    ////////////////////////////////////////////////////////////////////////////////////
    //Main Vl aisle only and Vl Secondary Aisles
    List<Point3d> P2 = new List<Point3d>();
    List<Point3d> MP2 = new List<Point3d>();
    List<double> valcheck = new List<double>();
    //    List<Rectangle3d> R2 = new List<Rectangle3d>();
    if (NumOfVlAisles > 0 &&
      NumOfHzAisles == 0 &&
      MainVlAisleW > 0 &&
      MainHzAisleW == 0)
    {
      Point3d StP0 = new Point3d(OPoint.X + 0.66 + BorderBoothW,
        OPoint.Y + EntranceAreaD,
        OPoint.Z);
      Point3d midP = new Point3d(OPoint.X + W * 0.5,
        OPoint.Y + EntranceAreaD,
        OPoint.Z);
      Point3d EndP2 = new Point3d(OPoint.X + (W - (0.66 + BorderBoothW)),
        OPoint.Y + (D - 0.24 - BorderBoothW - BackAreaD),
        OPoint.Z);
      Point3d midP1 = new Point3d(midP.X - (MainVlAisleW * 0.5),
        OPoint.Y + (D - 0.24 - BorderBoothW - BackAreaD),
        midP.Z);
      Point3d midP2 = new Point3d(midP.X + (MainVlAisleW * 0.5),
        midP.Y,
        midP.Z);
      P0.Add(StP0);
      P0.Add(midP1);
      P0.Add(midP);
      P0.Add(midP2);
      P0.Add(EndP2);
      Rectangle3d rec1 = new Rectangle3d(MainPlane, StP0, midP1);
      Rectangle3d rec2 = new Rectangle3d(MainPlane, midP2, EndP2);
      R0.Add(rec1);
      R0.Add(rec2);

      if (NumOfVlAisles % 2 == 0 && NumOfVlAisles > 3)
      {
        //Create Vl Aisles in Right side
        Point3d p0 = rec1.PointAt(0);
        Point3d p1 = rec1.PointAt(1);
        Point3d p2 = rec1.PointAt(2);
        Point3d p3 = rec1.PointAt(3);
        P0.Add(p1);
        P0.Add(p2);
        P0.Add(p3);

        double t = 1 / Convert.ToDouble(NumOfVlAisles * 0.5);
        double Vd = p0.DistanceTo(p3);
        double Hd = p0.DistanceTo(p1);
        double BW = (Hd / (NumOfVlAisles * 0.5)) - (AisleWidth);

        valcheck.Add(t);
        valcheck.Add(Vd);
        valcheck.Add(Hd);
        valcheck.Add(BW);
        for (int i = 1; i < NumOfVlAisles * 0.5 + 1; i++)
        {
          Point3d p00 = rec1.PointAt(t * i);
          Plane pp = new Plane(p00, NormVec);
          Rectangle3d rr = new Rectangle3d(pp, -BW, Vd);
          PV.Add(p00);
          VlB.Add(rr);
        }
        //Create Vl Aisles in Left side

        for (int i = 0; i < NumOfVlAisles * 0.5 ; i++)
        {
          Point3d p00 = rec2.PointAt(t * i);
          Plane pp = new Plane(p00, NormVec);
          Rectangle3d rr = new Rectangle3d(pp, BW, Vd);
          PV.Add(p00);
          VlB.Add(rr);
        }
      }
      else
      {
        Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
          "Number of vertical aisles has to be an even number and greater than 4");
      }
    }
    /////////////////////////////////////////////////////////////////////////////////////////////
    //Main Vl aisle only and Vl + Hz Secondary Aisles
    List<Point3d> P3 = new List<Point3d>();
    List<Point3d> LeftP3 = new List<Point3d>();
    List<Point3d> RightP3 = new List<Point3d>();
    List<Point3d> LeftPts = new List<Point3d>();
    List<Rectangle3d> LeftR3 = new List<Rectangle3d>();
    List<Point3d> RightPts = new List<Point3d>();
    List<Rectangle3d> RightR3 = new List<Rectangle3d>();

    if (NumOfVlAisles > 0 &&
      NumOfHzAisles > 0 &&
      MainVlAisleW > 0 &&
      MainHzAisleW == 0)
    {
      //Create Left and Right Rectangles
      Point3d midP = new Point3d(OPoint.X + W * 0.5,
        OPoint.Y + (EntranceAreaD - AisleWidth),
        OPoint.Z);
      Point3d LEP3D = new Point3d(OPoint.X + 0.8 + BorderBoothW,
        OPoint.Y + (EntranceAreaD - AisleWidth),
        OPoint.Z);
      Point3d LEP3U = new Point3d (LEP3D.X,
        OPoint.Y + (D - 0.24 - BorderBoothW - BackAreaD),
        midP.Z);
      Point3d REP3D = new Point3d(midP.X + (MainVlAisleW * 0.5),
        OPoint.Y + (EntranceAreaD - AisleWidth),
        midP.Z);
      Point3d REP3U = new Point3d(midP.X + (MainVlAisleW * 0.5),
        OPoint.Y + (D - 0.24 - BorderBoothW - BackAreaD),
        OPoint.Z);

      P3.Add(midP); // Mid Point of Vl Aisle

      LeftP3.Add(LEP3D);
      RightP3.Add(REP3D);

      //Create Hz Aisles
      //Left Side
      double dis = LEP3D.DistanceTo(LEP3U);
      double Bh3 = (dis / (NumOfHzAisles + 1)) - AisleWidth;

      for (int i = 0 ; i < NumOfHzAisles + 2 ; i++)
      {
        double xv = (dis / (NumOfHzAisles + 1)) * i;
        Point3d p = new Point3d(LEP3D.X, LEP3D.Y + (xv + AisleWidth), LEP3D.Z);
        Plane pl = new Plane(p, NormVec);
        LeftPts.Add(p);
      }

      for (int i = 0; i < LeftPts.Count - 1; i++)
      {
        Plane pl = new Plane(LeftPts[i], NormVec);
        Rectangle3d rec = new Rectangle3d(pl,
          (W * 0.5) - (MainVlAisleW * 0.5 + BorderBoothW + 0.8),
          Bh3);
        R0.Add(rec);
        LeftR3.Add(rec);
      }

      //Right Side
      for (int i = 0 ; i < NumOfHzAisles + 2 ; i++)
      {
        double xv = (dis / (NumOfHzAisles + 1)) * i;
        Point3d p = new Point3d(REP3D.X, REP3D.Y + (xv + AisleWidth), REP3D.Z);
        Plane pl = new Plane(p, NormVec);
        RightPts.Add(p);
      }

      for (int i = 0; i < RightPts.Count - 1; i++)
      {
        Plane pl = new Plane(RightPts[i], NormVec);
        Rectangle3d rec = new Rectangle3d(pl,
          (W * 0.5) - (MainVlAisleW * 0.5 + BorderBoothW + 0.8),
          Bh3);
        R0.Add(rec);
        RightR3.Add(rec);
      }

      //Create Vl Aisle
      if (NumOfVlAisles % 2 == 0 && NumOfVlAisles > 3)
      {
        //Left Side
        foreach (Rectangle3d r in LeftR3)
        {
          Point3d p0 = r.PointAt(0);
          Point3d p1 = r.PointAt(1);
          Point3d p2 = r.PointAt(2);
          Point3d p3 = r.PointAt(3);

          double t = 1 / Convert.ToDouble(NumOfVlAisles * 0.5);
          double Vd = p0.DistanceTo(p3);
          double Hd = p0.DistanceTo(p1);
          double BW = (Hd / (NumOfVlAisles * 0.5)) - (AisleWidth);

          for (int i = 1; i < NumOfVlAisles * 0.5 + 1; i++)
          {
            Point3d p00 = r.PointAt(t * i);
            Plane pp = new Plane(p00, NormVec);
            Rectangle3d rr = new Rectangle3d(pp, -BW, Vd);
            PV.Add(p00);
            VlB.Add(rr);
          }
        }

        //Right Side
        foreach (Rectangle3d r in RightR3)
        {
          Point3d p0 = r.PointAt(0);
          Point3d p1 = r.PointAt(1);
          Point3d p2 = r.PointAt(2);
          Point3d p3 = r.PointAt(3);

          double t = 1 / Convert.ToDouble(NumOfVlAisles * 0.5);
          double Vd = p0.DistanceTo(p3);
          double Hd = p0.DistanceTo(p1);
          double BW = (Hd / (NumOfVlAisles * 0.5)) - (AisleWidth);

          for (int i = 0; i < NumOfVlAisles * 0.5 ; i++)
          {
            Point3d p00 = r.PointAt(t * i);
            Plane pp = new Plane(p00, NormVec);
            Rectangle3d rr = new Rectangle3d(pp, BW, Vd);
            PV.Add(p00);
            VlB.Add(rr);
          }
        }
      }
      else
      {
        Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
          "Number of vertical aisles has to be an even number and greater than 4");
      }
    }
    //////////////////////////////////////////////////////////////////////////////////
    //Main Vl and Hz aisles + Vl Secondary aisles
    List<Point3d> P5 = new List<Point3d>();
    List<Point3d> MP5 = new List<Point3d>();
    //    List<Rectangle3d> R5 = new List<Rectangle3d>();

    if (NumOfVlAisles > 0 &&
      NumOfHzAisles == 0 &&
      MainVlAisleW > 0 &&
      MainHzAisleW > 0)
    {
      //Main Points
      Point3d StP5 = new Point3d(OPoint.X + 0.4 + BorderBoothW,
        OPoint.Y + EntranceAreaD,
        OPoint.Z);
      Point3d UEnP5 = new Point3d(OPoint.X + 0.4 + BorderBoothW,
        OPoint.Y + (D - 0.24 - BorderBoothW - BackAreaD),
        OPoint.Z);
      Point3d DEnP5 = new Point3d((W - (0.4 + BorderBoothW )),
        OPoint.Y + EntranceAreaD,
        OPoint.Z);

      //Vertical Aisle Points
      Point3d VMidP5 = new Point3d(OPoint.X + (W * 0.5),
        OPoint.Y + EntranceAreaD,
        OPoint.Z);

      //Horizontal Aisle Points
      double dis = (StP5.DistanceTo(UEnP5) * 0.5) + EntranceAreaD; //this is wrong! fix this distance!
      Point3d HMidP5 = new Point3d(OPoint.X + 0.12 + BorderBoothW + AisleWidth * 0.5,
        OPoint.Y + dis,
        OPoint.Z);

      //First Quarter
      Point3d LVMidP5 = new Point3d(VMidP5.X - (MainVlAisleW * 0.5),
        HMidP5.Y - (0.5 * MainHzAisleW),
        OPoint.Z);
      Rectangle3d recQ1 = new Rectangle3d(MainPlane, StP5, LVMidP5);
      R0.Add(recQ1);
      LeftR3.Add(recQ1);

      //Second Quarter
      Point3d RVMidP5 = new Point3d(VMidP5.X + (MainVlAisleW * 0.5),
        HMidP5.Y - (0.5 * MainHzAisleW),
        OPoint.Z);
      Rectangle3d recQ2 = new Rectangle3d(MainPlane, DEnP5, RVMidP5);
      R0.Add(recQ2);
      RightR3.Add(recQ2);

      //Third Quarter
      Point3d LUHMidP5 = new Point3d(OPoint.X + 0.4 + BorderBoothW,
        HMidP5.Y + (0.5 * MainHzAisleW),
        OPoint.Z);
      Point3d LUVMidP5 = new Point3d(VMidP5.X - (MainVlAisleW * 0.5),
        OPoint.Y + (D - 0.24 - BorderBoothW - BackAreaD),
        OPoint.Z);
      Rectangle3d recQ3 = new Rectangle3d(MainPlane, LUHMidP5, LUVMidP5);
      R0.Add(recQ3);
      LeftR3.Add(recQ3);

      //Fourth Quarter
      Point3d RUHMidP5 = new Point3d(VMidP5.X + (MainVlAisleW * 0.5),
        HMidP5.Y + (0.5 * MainHzAisleW),
        OPoint.Z);
      Point3d RUVMidP5 = new Point3d((W - (0.4 + BorderBoothW  )),
        OPoint.Y + (D - 0.24 - BorderBoothW - BackAreaD),
        OPoint.Z);
      Rectangle3d recQ4 = new Rectangle3d(MainPlane, RUHMidP5, RUVMidP5);
      R0.Add(recQ4);
      RightR3.Add(recQ4);

      P5.Add(StP5);
      P5.Add(VMidP5);
      P5.Add(LVMidP5);
      P5.Add(RVMidP5);
      P5.Add(DEnP5);
      P5.Add(HMidP5);
      P5.Add(LUHMidP5);
      P5.Add(RUHMidP5);
      P5.Add(LUVMidP5);
      P5.Add(RUVMidP5);

      //Create Vl Aisles
      if (NumOfVlAisles % 2 == 0 && NumOfVlAisles > 3)
      {
        //Left Side
        foreach (Rectangle3d r in LeftR3)
        {
          Point3d p0 = r.PointAt(0);
          Point3d p1 = r.PointAt(1);
          Point3d p2 = r.PointAt(2);
          Point3d p3 = r.PointAt(3);

          double t = 1 / Convert.ToDouble(NumOfVlAisles * 0.5);
          double Vd = p0.DistanceTo(p3);
          double Hd = p0.DistanceTo(p1);
          double BW = (Hd / (NumOfVlAisles * 0.5)) - (AisleWidth);

          for (int i = 1; i < NumOfVlAisles * 0.5 + 1; i++)
          {
            Point3d p00 = r.PointAt(t * i);
            Plane pp = new Plane(p00, NormVec);
            Rectangle3d rr = new Rectangle3d(pp, -BW, Vd);
            PV.Add(p00);
            VlB.Add(rr);
          }
        }

        //Right Side
        foreach (Rectangle3d r in RightR3)
        {
          Point3d p0 = r.PointAt(0);
          Point3d p1 = r.PointAt(1);
          Point3d p2 = r.PointAt(2);
          Point3d p3 = r.PointAt(3);

          double t = 1 / Convert.ToDouble(NumOfVlAisles * 0.5);
          double Vd = p0.DistanceTo(p3);
          double Hd = p0.DistanceTo(p1);
          double BW = (Hd / (NumOfVlAisles * 0.5)) - (AisleWidth);

          for (int i = 0; i < NumOfVlAisles * 0.5 ; i++)
          {
            Point3d p00 = r.PointAt(t * i);
            Plane pp = new Plane(p00, NormVec);
            Rectangle3d rr = new Rectangle3d(pp, BW, Vd);
            PV.Add(p00);
            VlB.Add(rr);
          }
        }
      }
      else
      {
        Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
          "Number of vertical aisles has to be an even number and greater than 4");
      }
    }
    ///////////////////////////////////////////////////////////////////////////////
    //Hz Secondary aisles only
    if (NumOfVlAisles == 0 &&
      NumOfHzAisles > 0 &&
      MainVlAisleW == 0 &&
      MainHzAisleW == 0)
    {
      Point3d StP0 = new Point3d(OPoint.X + 0.4 + BorderBoothW + AisleWidth,
        OPoint.Y + EntranceAreaD,
        OPoint.Z);
      Point3d EndP2 = new Point3d(OPoint.X + (W - (0.4 + BorderBoothW + AisleWidth)),
        OPoint.Y + (D - 0.24 - BackAreaD),
        OPoint.Z);
      //      P0.Add(StP0);
      //      P0.Add(EndP2);
      Rectangle3d rec = new Rectangle3d(MainPlane, StP0, EndP2);
      R0.Add(rec);
      Point3d StP1 = rec.PointAt(1);
      Point3d StP3 = rec.PointAt(3);
      //      P0.Add(StP1);


      //Create Vl Aisles
      double t = 1 / Convert.ToDouble(NumOfHzAisles);
      double Vd = StP0.DistanceTo(StP3);
      double Hd = StP0.DistanceTo(StP1);
      double BW = (Vd / NumOfHzAisles) - (AisleWidth);

      foreach(Rectangle3d r in R0)
      {
        Point3d p0 = r.PointAt(0.0);
        P0.Add(p0);
        for (int i = 1; i < NumOfHzAisles; i++)
        {
          Point3d p1 = r.PointAt((t * i) + 3);
          P0.Add(p1);
          Plane pp0 = new Plane(p0, NormVec);
          Plane pp1 = new Plane(p1, NormVec);
          Rectangle3d rec0 = new Rectangle3d(pp0, Hd, BW);
          Rectangle3d rec1 = new Rectangle3d(pp1, Hd, BW);
          VlB.Add(rec0);
          VlB.Add(rec1);
        }
      }
    }
    ///////////////////////////////////////////////////////////////////////////////
    //Hz Main and Secondary aisles
    if (NumOfVlAisles == 0 &&
      NumOfHzAisles > 0 &&
      MainVlAisleW == 0 &&
      MainHzAisleW > 0)
    {
      Point3d StP0 = new Point3d(OPoint.X + 0.4 + BorderBoothW + AisleWidth,
        OPoint.Y + (EntranceAreaD - AisleWidth),
        0.0);
      Point3d EndP0 = new Point3d(OPoint.X + (W - (0.4 + BorderBoothW + AisleWidth)),
        OPoint.Y + (D - 0.24 - BackAreaD),
        0.0);
      P0.Add(StP0);
      P0.Add(EndP0);
      Rectangle3d rec = new Rectangle3d(MainPlane, StP0, EndP0);
      R0.Add(rec);

      Point3d StP1 = rec.PointAt(1);
      Point3d StP3 = rec.PointAt(3);
      P0.Add(StP1);
      P0.Add(StP3);
      double Hd = StP0.DistanceTo(StP1);
      double Vd2 = StP0.DistanceTo(StP3);
      double MidDis = Vd2 * 0.5;
      Point3d MidP0 = new Point3d(StP0.X,
        StP0.Y + MidDis,
        0.0);
      Point3d MidP1 = new Point3d(StP0.X,
        StP0.Y + MidDis + (MainHzAisleW * 0.5),
        0.0);
      Point3d MidP2 = new Point3d(StP0.X,
        StP0.Y + MidDis - (MainHzAisleW * 0.5),
        0.0);
      Point3d EndP2 = new Point3d(OPoint.X + (W - (0.4 + BorderBoothW + AisleWidth)),
        StP0.Y + MidDis - (MainHzAisleW * 0.5),
        0.0);
      P0.Add(MidP0);
      P0.Add(MidP1);
      P0.Add(MidP2);
      P0.Add(EndP2);


      Plane pp0 = new Plane(StP0, NormVec);
      Plane pp1 = new Plane(MidP2, NormVec);
      Rectangle3d recDown = new Rectangle3d(pp0, StP0, EndP2);
      Rectangle3d recUp = new Rectangle3d(pp0, MidP1, EndP0);
      R0.Add(recDown);
      R0.Add(recUp);
      //Create Hl Aisles
      if (NumOfHzAisles % 2 == 0 && NumOfHzAisles > 3)
      {
        Point3d p0 = recUp.PointAt(0);
        Point3d p1 = recUp.PointAt(1);
        Point3d p2 = recUp.PointAt(2);
        Point3d p3 = recUp.PointAt(3);

        double t = 1 / Convert.ToDouble(NumOfHzAisles * 0.5);
        double VdPerHalf = p0.DistanceTo(p3);
        double BW = (VdPerHalf / (NumOfHzAisles * 0.5)) - (AisleWidth);

        //Upside Booths Space
        for (int i = 1; i < (NumOfHzAisles * 0.5); i++)
        {
          Point3d UpMidPt0 = recUp.PointAt(0.0);
          Point3d UpMidPt1 = recUp.PointAt((t * i) + 3);
          Plane Upp0 = new Plane (UpMidPt0, NormVec);
          Plane Upp1 = new Plane (UpMidPt1, NormVec);
          Rectangle3d UpRecBooths0 = new Rectangle3d(Upp0, Hd, BW);
          Rectangle3d UpRecBooths1 = new Rectangle3d(Upp1, Hd, BW);
          VlB.Add(UpRecBooths0);
          VlB.Add(UpRecBooths1);
        }

        //Downside Booths Space
        for (int i = 1; i < (NumOfHzAisles * 0.5); i++)
        {
          Point3d DownMidPt0 = recDown.PointAt(3.0);
          Point3d DownMidPt1 = recDown.PointAt((t * i) + 3);
          Plane Downp0 = new Plane (DownMidPt0, NormVec);
          Plane Downp1 = new Plane (DownMidPt1, NormVec);
          Rectangle3d DownRecBooths0 = new Rectangle3d(Downp0, Hd, -BW);
          Rectangle3d DownRecBooths1 = new Rectangle3d(Downp1, Hd, -BW);
          VlB.Add(DownRecBooths0);
          VlB.Add(DownRecBooths1);
        }
      }
      else
      {
        Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
          "Number of horizontal aisles has to be an even number and greater than 4");
      }
    }
    //////////////////////////////////////////////////////////////////////////////////////////
    //Hz Main and Vl Secondary aisles
    if (NumOfVlAisles > 0 &&
      NumOfHzAisles == 0 &&
      MainVlAisleW == 0 &&
      MainHzAisleW > 0)
    {
      Point3d StP0 = new Point3d(OPoint.X + (0.4 + BorderBoothW + (AisleWidth * 0.5)),
        OPoint.Y + EntranceAreaD,
        0.0);
      Point3d EndP0 = new Point3d(OPoint.X + (W - (0.4 + BorderBoothW + ( AisleWidth * 0.5))),
        OPoint.Y + D - (0.4 + BorderBoothW + BackAreaD),
        0.0);
      P0.Add(StP0);
      P0.Add(EndP0);
      Rectangle3d rec = new Rectangle3d(MainPlane, StP0, EndP0);
      R0.Add(rec);

      Point3d StP1 = rec.PointAt(1);
      Point3d StP3 = rec.PointAt(3);
      P0.Add(StP1);
      P0.Add(StP3);
      double Hd = StP0.DistanceTo(StP1);
      double Vd2 = StP0.DistanceTo(StP3);
      double MidDis = Vd2 * 0.5;
      Point3d MidP0 = new Point3d(StP0.X,
        StP0.Y + MidDis,
        0.0);
      Point3d MidP1 = new Point3d(StP0.X,
        StP0.Y + MidDis + (MainHzAisleW * 0.5),
        0.0);
      Point3d MidP2 = new Point3d(StP0.X,
        StP0.Y + MidDis - (MainHzAisleW * 0.5),
        0.0);
      Point3d EndP2 = new Point3d(OPoint.X + (W - (0.4 + BorderBoothW + ( AisleWidth * 0.5))),
        StP0.Y + MidDis - (MainHzAisleW * 0.5),
        0.0);
      P0.Add(MidP0);
      P0.Add(MidP1);
      P0.Add(MidP2);
      P0.Add(EndP2);

      Plane pp0 = new Plane(StP0, NormVec);
      Plane pp1 = new Plane(MidP2, NormVec);
      Rectangle3d recDown = new Rectangle3d(pp0, StP0, EndP2);
      Rectangle3d recUp = new Rectangle3d(pp0, MidP1, EndP0);
      R0.Add(recDown);
      R0.Add(recUp);

      //Create Vl Aisles

      Point3d p0 = recUp.PointAt(0);
      Point3d p1 = recUp.PointAt(1);
      Point3d p2 = recUp.PointAt(2);
      Point3d p3 = recUp.PointAt(3);

      double t = 1 / Convert.ToDouble(NumOfVlAisles);
      double VdPerHalf = p0.DistanceTo(p3);
      double BW = (Hd / NumOfVlAisles ) - (AisleWidth);


      //Upside Booths Space
      for (int i = 1; i < NumOfVlAisles + 1; i++)
      {
        Point3d Up0 = recUp.PointAt(t * i);
        Point3d Up1 = new Point3d(Up0.X - (0.5 * AisleWidth), Up0.Y, 0.0);
        Plane Upp = new Plane(Up1, NormVec);
        Rectangle3d Urr = new Rectangle3d(Upp, -BW, VdPerHalf);
        PV.Add(Up1);
        VlB.Add(Urr);
      }

      //Downside Booths Space
      for (int i = 1; i < NumOfVlAisles + 1; i++)
      {
        Point3d Up0 = recDown.PointAt(t * i);
        Point3d Up1 = new Point3d(Up0.X - (0.5 * AisleWidth), Up0.Y, 0.0);
        Plane Upp = new Plane(Up1, NormVec);
        Rectangle3d Urr = new Rectangle3d(Upp, -BW, VdPerHalf);
        PV.Add(Up1);
        VlB.Add(Urr);
      }
    }
    //////////////////////////////////////////////////////////////////////////////////////////
    //Vl Main and Hz Secondary aisles
    if (NumOfVlAisles == 0 &&
      NumOfHzAisles > 0 &&
      MainVlAisleW > 0 &&
      MainHzAisleW == 0)
    {
      Point3d StP0 = new Point3d(OPoint.X + 0.4 + BorderBoothW + AisleWidth,
        OPoint.Y + EntranceAreaD,
        OPoint.Z);
      Point3d midP = new Point3d(OPoint.X + W * 0.5,
        OPoint.Y + EntranceAreaD,
        OPoint.Z);
      Point3d EndP2 = new Point3d(OPoint.X + (W - (0.66 + BorderBoothW)),
        OPoint.Y + (D - 0.24 - BorderBoothW - BackAreaD + AisleWidth),
        OPoint.Z);
      Point3d midP1 = new Point3d(midP.X - (MainVlAisleW * 0.5),
        OPoint.Y + (D - 0.24 - BorderBoothW - BackAreaD + AisleWidth),
        midP.Z);
      Point3d midP2 = new Point3d(midP.X + (MainVlAisleW * 0.5),
        midP.Y,
        midP.Z);
      P0.Add(StP0);
      P0.Add(midP1);
      P0.Add(midP);
      P0.Add(midP2);
      P0.Add(EndP2);
      Rectangle3d rec1 = new Rectangle3d(MainPlane, StP0, midP1);
      Rectangle3d rec2 = new Rectangle3d(MainPlane, midP2, EndP2);
      R0.Add(rec1);
      R0.Add(rec2);

      //Hz Aisles
      Point3d p0 = rec1.PointAt(0.0);
      Point3d p1 = rec1.PointAt(1.0);
      Point3d p2 = rec1.PointAt(2.0);
      Point3d p3 = rec1.PointAt(3.0);
      double t = 1 / Convert.ToDouble(NumOfHzAisles);
      double Vd = p0.DistanceTo(p3);
      double HdPerHalf = p0.DistanceTo(p1);
      double BW = (Vd / (NumOfHzAisles) ) - (AisleWidth);

      //At Left Side
      for (int i = 1; i < NumOfHzAisles; i++)
      {
        Point3d LMidPt0 = rec1.PointAt(0.0);
        Point3d LMidPt1 = rec1.PointAt((t * i) + 3);
        PV.Add(LMidPt1);
        Plane Lp0 = new Plane (LMidPt0, NormVec);
        Plane Lp1 = new Plane (LMidPt1, NormVec);
        Rectangle3d LRecBooths0 = new Rectangle3d(Lp0, HdPerHalf, BW);
        Rectangle3d LRecBooths1 = new Rectangle3d(Lp1, HdPerHalf, BW);
        VlB.Add(LRecBooths0);
        VlB.Add(LRecBooths1);
      }

      //At Right Side
      for (int i = 1; i < NumOfHzAisles; i++)
      {
        Point3d RMidPt0 = rec2.PointAt(0.0);
        Point3d RMidPt1 = rec2.PointAt((t * i) + 3);
        PV.Add(RMidPt1);
        Plane Rp0 = new Plane (RMidPt0, NormVec);
        Plane Rp1 = new Plane (RMidPt1, NormVec);
        Rectangle3d RRecBooths0 = new Rectangle3d(Rp0, HdPerHalf, BW);
        Rectangle3d RRecBooths1 = new Rectangle3d(Rp1, HdPerHalf, BW);
        VlB.Add(RRecBooths0);
        VlB.Add(RRecBooths1);
      }
    }
    ////////////////////////////////////////////////
    //If Hz and Vl main + Hz secondary are true
    if (NumOfVlAisles == 0 &&
      NumOfHzAisles > 0 &&
      MainVlAisleW > 0 &&
      MainHzAisleW > 0)
    {
      //Main Points
      Point3d StP5 = new Point3d(OPoint.X + 0.4 + BorderBoothW,
        OPoint.Y + (EntranceAreaD - AisleWidth),
        OPoint.Z);
      Point3d UEnP5 = new Point3d(OPoint.X + 0.4 + BorderBoothW,
        OPoint.Y + (D - 0.24 - BorderBoothW - BackAreaD + AisleWidth),
        OPoint.Z);
      Point3d DEnP5 = new Point3d((W - (0.4 + BorderBoothW )),
        OPoint.Y + (EntranceAreaD - AisleWidth),
        OPoint.Z);
      double TotalVlDis = StP5.DistanceTo(UEnP5) / 2;
      //Vertical Aisle Points
      Point3d VMidP5 = new Point3d(OPoint.X + (W * 0.5),
        OPoint.Y + EntranceAreaD,
        OPoint.Z);

      //Horizontal Aisle Points
      double dis = (StP5.DistanceTo(UEnP5) * 0.5) + EntranceAreaD; //this is wrong! fix this distance!
      Point3d HMidP5 = new Point3d(OPoint.X + 0.12 + BorderBoothW + AisleWidth * 0.5,
        StP5.Y + TotalVlDis,
        0.0);
      P0.Add(HMidP5);
      //First Quarter
      Point3d LVMidP5 = new Point3d(VMidP5.X - (MainVlAisleW * 0.5),
        HMidP5.Y - (0.5 * MainHzAisleW),
        OPoint.Z);
      Rectangle3d recQ1 = new Rectangle3d(MainPlane, StP5, LVMidP5);
      R0.Add(recQ1);
      LeftR3.Add(recQ1);
      //Adding Hz Aisles
      Point3d PQ1 = recQ1.PointAt(1.0);
      Point3d PQ3 = recQ1.PointAt(3.0);

      double HdperQ = StP5.DistanceTo(PQ1) - AisleWidth;
      double VdperQ = StP5.DistanceTo(PQ3);
      double tHz = 1.00 / (NumOfHzAisles / 2);

      double HBW = (VdperQ / (NumOfHzAisles / 2)) - AisleWidth;
      for (int i = 0; i < NumOfHzAisles / 2; i++)
      {
        Point3d pQ1 = recQ1.PointAt(2 - (tHz * i));
        Plane ppQ1 = new Plane(pQ1, NormVec);
        Rectangle3d HRecQ1 = new Rectangle3d(ppQ1, -HdperQ, -HBW);
        P0.Add(pQ1);
        VlB.Add(HRecQ1);
      }

      //Second Quarter
      Point3d RVMidP5 = new Point3d(VMidP5.X + (MainVlAisleW * 0.5),
        HMidP5.Y - (0.5 * MainHzAisleW),
        OPoint.Z);
      Rectangle3d recQ2 = new Rectangle3d(MainPlane, DEnP5, RVMidP5);
      R0.Add(recQ2);
      RightR3.Add(recQ2);
      for (int i = 0; i < NumOfHzAisles / 2; i++)
      {
        Point3d pQ2 = recQ2.PointAt(3 + (tHz * i));
        Plane ppQ2 = new Plane(pQ2, NormVec);
        Rectangle3d HRecQ2 = new Rectangle3d(ppQ2, HdperQ, -HBW);
        P0.Add(pQ2);
        VlB.Add(HRecQ2);
      }

      //Third Quarter
      Point3d LUHMidP5 = new Point3d(OPoint.X + 0.4 + BorderBoothW,
        HMidP5.Y + (0.5 * MainHzAisleW),
        OPoint.Z);
      Point3d LUVMidP5 = new Point3d(VMidP5.X - (MainVlAisleW * 0.5),
        OPoint.Y + (D - 0.24 - BorderBoothW - BackAreaD + AisleWidth),
        OPoint.Z);
      Rectangle3d recQ3 = new Rectangle3d(MainPlane, LUHMidP5, LUVMidP5);
      R0.Add(recQ3);
      LeftR3.Add(recQ3);
      for (int i = 0; i < NumOfHzAisles / 2; i++)
      {
        Point3d pQ3 = recQ3.PointAt(1 + (tHz * i));
        Plane ppQ3 = new Plane(pQ3, NormVec);
        Rectangle3d HRecQ3 = new Rectangle3d(ppQ3, -HdperQ, HBW);
        P0.Add(pQ3);
        VlB.Add(HRecQ3);
      }

      //Fourth Quarter
      Point3d RUHMidP5 = new Point3d(VMidP5.X + (MainVlAisleW * 0.5),
        HMidP5.Y + (0.5 * MainHzAisleW),
        OPoint.Z);
      Point3d RUVMidP5 = new Point3d((W - (0.4 + BorderBoothW  )),
        OPoint.Y + (D - 0.24 - BorderBoothW - BackAreaD + AisleWidth),
        OPoint.Z);
      Rectangle3d recQ4 = new Rectangle3d(MainPlane, RUHMidP5, RUVMidP5);
      R0.Add(recQ4);
      RightR3.Add(recQ4);
      for (int i = 0; i < NumOfHzAisles / 2; i++)
      {
        Point3d pQ4 = recQ4.PointAt(4 - (tHz * i));
        Plane ppQ4 = new Plane(pQ4, NormVec);
        Rectangle3d HRecQ4 = new Rectangle3d(ppQ4, HdperQ, HBW);
        P0.Add(pQ4);
        VlB.Add(HRecQ4);
      }
    }
    //////////////////////////////////////////////////////////////////////////////////////////
    //Main Vl, Hz Aisles and Secondary Vl and Hz aisles
    List<Rectangle3d> LHzSpcs = new List<Rectangle3d>();
    List<Rectangle3d> RHzSpcs = new List<Rectangle3d>();
    if (NumOfVlAisles > 0 &&
      NumOfHzAisles > 0 &&
      MainVlAisleW > 0 &&
      MainHzAisleW > 0)
    {
      //Main Points
      Point3d StP5 = new Point3d(OPoint.X + 0.4 + BorderBoothW,
        OPoint.Y + (EntranceAreaD - AisleWidth),
        OPoint.Z);
      Point3d UEnP5 = new Point3d(OPoint.X + 0.4 + BorderBoothW,
        OPoint.Y + (D - 0.24 - BorderBoothW - BackAreaD + AisleWidth),
        OPoint.Z);
      Point3d DEnP5 = new Point3d((W - (0.4 + BorderBoothW )),
        OPoint.Y + (EntranceAreaD - AisleWidth),
        OPoint.Z);
      double TotalVlDis = StP5.DistanceTo(UEnP5) / 2;
      //Vertical Aisle Points
      Point3d VMidP5 = new Point3d(OPoint.X + (W * 0.5),
        OPoint.Y + EntranceAreaD,
        OPoint.Z);

      //Horizontal Aisle Points
      double dis = (StP5.DistanceTo(UEnP5) * 0.5) + EntranceAreaD; //this is wrong! fix this distance!
      Point3d HMidP5 = new Point3d(OPoint.X + 0.12 + BorderBoothW + AisleWidth * 0.5,
        StP5.Y + TotalVlDis,
        0.0);
      P0.Add(HMidP5);
      //First Quarter
      Point3d LVMidP5 = new Point3d(VMidP5.X - (MainVlAisleW * 0.5),
        HMidP5.Y - (0.5 * MainHzAisleW),
        OPoint.Z);
      Rectangle3d recQ1 = new Rectangle3d(MainPlane, StP5, LVMidP5);
      R0.Add(recQ1);
      LeftR3.Add(recQ1);
      //Adding Hz Aisles
      Point3d PQ1 = recQ1.PointAt(1.0);
      Point3d PQ3 = recQ1.PointAt(3.0);

      double HdperQ = StP5.DistanceTo(PQ1);
      double VdperQ = StP5.DistanceTo(PQ3);
      double tHz = 1.00 / (NumOfHzAisles / 2);
      double tVl = 1.00 / (NumOfVlAisles / 2);

      double HBW = (VdperQ / (NumOfHzAisles / 2)) - AisleWidth;
      for (int i = 0; i < NumOfHzAisles / 2; i++)
      {
        Point3d pQ1 = recQ1.PointAt(2 - (tHz * i));
        Plane ppQ1 = new Plane(pQ1, NormVec);
        Rectangle3d HRecQ1 = new Rectangle3d(ppQ1, -HdperQ, -HBW);
        P0.Add(pQ1);
        RHzSpcs.Add(HRecQ1);
      }

      //Second Quarter
      Point3d RVMidP5 = new Point3d(VMidP5.X + (MainVlAisleW * 0.5),
        HMidP5.Y - (0.5 * MainHzAisleW),
        OPoint.Z);
      Rectangle3d recQ2 = new Rectangle3d(MainPlane, DEnP5, RVMidP5);
      R0.Add(recQ2);
      RightR3.Add(recQ2);
      for (int i = 0; i < NumOfHzAisles / 2; i++)
      {
        Point3d pQ2 = recQ2.PointAt(3 + (tHz * i));
        Plane ppQ2 = new Plane(pQ2, NormVec);
        Rectangle3d HRecQ2 = new Rectangle3d(ppQ2, HdperQ, -HBW);
        P0.Add(pQ2);
        LHzSpcs.Add(HRecQ2);
      }

      //Third Quarter
      Point3d LUHMidP5 = new Point3d(OPoint.X + 0.4 + BorderBoothW,
        HMidP5.Y + (0.5 * MainHzAisleW),
        OPoint.Z);
      Point3d LUVMidP5 = new Point3d(VMidP5.X - (MainVlAisleW * 0.5),
        OPoint.Y + (D - 0.24 - BorderBoothW - BackAreaD + AisleWidth),
        OPoint.Z);
      Rectangle3d recQ3 = new Rectangle3d(MainPlane, LUHMidP5, LUVMidP5);
      R0.Add(recQ3);
      LeftR3.Add(recQ3);
      for (int i = 0; i < NumOfHzAisles / 2; i++)
      {
        Point3d pQ3 = recQ3.PointAt(1 + (tHz * i));
        Plane ppQ3 = new Plane(pQ3, NormVec);
        Rectangle3d HRecQ3 = new Rectangle3d(ppQ3, -HdperQ, HBW);
        P0.Add(pQ3);
        RHzSpcs.Add(HRecQ3);
      }

      //Fourth Quarter
      Point3d RUHMidP5 = new Point3d(VMidP5.X + (MainVlAisleW * 0.5),
        HMidP5.Y + (0.5 * MainHzAisleW),
        OPoint.Z);
      Point3d RUVMidP5 = new Point3d((W - (0.4 + BorderBoothW  )),
        OPoint.Y + (D - 0.24 - BorderBoothW - BackAreaD + AisleWidth),
        OPoint.Z);
      Rectangle3d recQ4 = new Rectangle3d(MainPlane, RUHMidP5, RUVMidP5);
      R0.Add(recQ4);
      RightR3.Add(recQ4);
      for (int i = 0; i < NumOfHzAisles / 2; i++)
      {
        Point3d pQ4 = recQ4.PointAt(4 - (tHz * i));
        Plane ppQ4 = new Plane(pQ4, NormVec);
        Rectangle3d HRecQ4 = new Rectangle3d(ppQ4, HdperQ, HBW);
        P0.Add(pQ4);
        LHzSpcs.Add(HRecQ4);
      }

      //Adding Vl Aisles
      if (NumOfVlAisles % 2 == 0 && NumOfVlAisles > 3)
      {
        double vBW = (HdperQ / (NumOfVlAisles / 2)) - AisleWidth;
        //Left Side
        foreach (Rectangle3d r in LHzSpcs)
        {
          Point3d VpQ0 = r.PointAt(0);
          for (int i = 0; i < (NumOfVlAisles / 2);i++)
          {
            Point3d VpQ = r.PointAt(tVl * i);
            Plane Vpp = new Plane(VpQ, NormVec);
            Rectangle3d Vrec1 = new Rectangle3d (Vpp, vBW, HBW);
            PV.Add(VpQ);
            VlB.Add(Vrec1);
          }
        }
        //Right Side
        foreach (Rectangle3d r in RHzSpcs)
        {

          for (int i = 1; i < (NumOfVlAisles / 2);i++)
          {
            Point3d VpQ0 = r.PointAt(1);
            Point3d VpQ = r.PointAt(tVl * i);
            Plane Vpp0 = new Plane(VpQ0, NormVec);
            Plane Vpp = new Plane(VpQ, NormVec);
            Rectangle3d Vrec0 = new Rectangle3d (Vpp0, -vBW, HBW);
            Rectangle3d Vrec2 = new Rectangle3d (Vpp, -vBW, HBW);
            PV.Add(VpQ0);
            PV.Add(VpQ);
            VlB.Add(Vrec0);
            VlB.Add(Vrec2);
          }
        }
      }
      else
      {
        Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
          "Number of horizontal aisles has to be an even number and greater than 4");
      }
    }



    //Output

    Main_Boundary_Rectangle = R0;

    Exhibition_Boundary_Rectangle = VlB;

    //Component d
/Algorithm
    //Central Area Booths
    //Find number of booths per column
    Vector3d NormVec = new Vector3d (0.0, 0.0, 1.0);
    Plane MainPlane = new Plane(OPoint, NormVec);

    double boothwidth = new double ();
    double boothlength = new double ();
    List < Point3d > Pts = new List<Point3d>();
    List < Point3d > EdgePts = new List<Point3d>();
    List < Rectangle3d > BRec = new List<Rectangle3d>();
    List <double> val = new List <double>();
    List < Curve > Crv = new List< Curve >();
    DataTree<Point3d> THMidPts = new DataTree<Point3d >();
    DataTree<Point3d> EPts = new DataTree<Point3d >();
    ///////////////////////////////////////////////////////////
    //In Case of Hz Aisle only
    if (SecVlAisles == 0
      && MainHzAisle >= 0
      && SecHzAisles > 0
      && MainVlAisle >= 0
      )
    {


      int HNumOfSpc = SecHzAisles;
      int HNumOfBoothsPerSpc = NumOfCentralBooth / HNumOfSpc;
      if (MainVlAisle > 0)
      {
        HNumOfBoothsPerSpc = HNumOfBoothsPerSpc / 2;
      }
      else
      {
        HNumOfBoothsPerSpc = HNumOfBoothsPerSpc + 0;
      }

      int arrayH = Convert.ToInt32(HNumOfBoothsPerSpc);
      Point3d[] MidPts = new Point3d[arrayH];
      int pathindex = 0;

      foreach(List<Rectangle3d> rec in CentralBoothCol.Branches)
      {
        List <Point3d> PtList = new List<Point3d>();
        List <Point3d> HMidPts = new List<Point3d>();
        List <Point3d> EntPts = new List<Point3d>();
        GH_Path path = new GH_Path(pathindex);
        pathindex = pathindex + 1;
        foreach (Rectangle3d r0 in rec)
        {
          Point3d p00 = r0.PointAt(0.0);
          Point3d p11 = r0.PointAt(1.0);
          Point3d p33 = r0.PointAt(3.0);
          PtList.Add(p00);
          PtList.Add(p11);
          PtList.Add(p33);
          double vld = p00.DistanceTo(p33);
          double BW = p00.DistanceTo(p11) / HNumOfBoothsPerSpc;
          boothwidth = BW;
          boothlength = vld;

          for (int i = 0; i < HNumOfBoothsPerSpc; i++)
          {
            Point3d Hpt = new Point3d(p00.X + (BW * i), p00.Y, 0.0);
            Point3d enPt = new Point3d(p00.X + (BW * i) + (BW * 0.5), p00.Y - 1.0, 0.0);
            PtList.Add(Hpt);
            Plane Hpp = new Plane(Hpt, NormVec);
            Rectangle3d HRec = new Rectangle3d(Hpp, BW, vld);
            BRec.Add(HRec);
            EntPts.Add(enPt);
          }
        }
        THMidPts.AddRange(PtList, path);
        EPts.AddRange(EntPts, path);
      }
    }
      ///////////////////////////////////////////////////////////////
    else
    {
      //In Case of Both Vl and Hz Aisles or Vl Aisles only
      int HzAisles = new int ();

      if (MainHzAisle > 0)
      {
        HzAisles = 2;
      }
      else
      {
        HzAisles = SecHzAisles + 1;
      }

      if(SecHzAisles == 0)
      {
        SecHzAisles = 1;
      }
      if(SecVlAisles == 0)
      {
        SecVlAisles = 1;
      }
      int HNumOfSpc = SecVlAisles * (SecHzAisles);

      int HNumOfBoothsPerSpc = NumOfCentralBooth / HNumOfSpc;
      if(HNumOfBoothsPerSpc <= 5 && HNumOfBoothsPerSpc > 2)
      {

        HNumOfBoothsPerSpc = 4;
        int nH0 = HNumOfBoothsPerSpc / 2;
        int arrayH0 = Convert.ToInt32(nH0);
        Point3d[] MidPts0 = new Point3d[arrayH0];
        int pathindex0 = 0;
        foreach(List<Rectangle3d> rec in CentralBoothCol.Branches)
        {
          List <Point3d> PtList = new List<Point3d>();
          List <Point3d> HMidPts = new List<Point3d>();
          List <Point3d> EntPts = new List<Point3d>();

          GH_Path path0 = new GH_Path(pathindex0);
          pathindex0 = pathindex0 + 1;
          foreach (Rectangle3d r0 in rec)
          {
            Point3d p00 = r0.PointAt(0.5);
            Point3d p11 = r0.PointAt(2.5);
            Point3d p22 = r0.PointAt(0.0);
            Point3d p33 = r0.PointAt(1.0);

            PtList.Add(p00);
            double vld0 = p00.DistanceTo(p11) / 2;
            double hd0 = p22.DistanceTo(p33) * 0.5;
            boothwidth = hd0;
            boothlength = vld0;
            Point3d mp00 = new Point3d(p00.X, p00.Y + vld0, 0.0);
            Point3d EP_right00 = new Point3d(p00.X - hd0 - 0.5, p00.Y + (vld0 * 1.5), 0.0);
            Point3d EP_left00 = new Point3d(p00.X + hd0 + 0.5, p00.Y + (vld0 * 1.5), 0.0);
            Point3d EP_right11 = new Point3d(p00.X - hd0 - 0.5, p00.Y + (vld0 * 0.5), 0.0);
            Point3d EP_left11 = new Point3d(p00.X + hd0 + 0.5, p00.Y + (vld0 * 0.5), 0.0);
            PtList.Add(mp00);
            Plane pp00 = new Plane(p00, NormVec);
            Plane mpp00 = new Plane(mp00, NormVec);
            Rectangle3d Rrec00 = new Rectangle3d(pp00, hd0, vld0);
            Rectangle3d Rmrec00 = new Rectangle3d(mpp00, hd0, vld0);
            Rectangle3d Lrec00 = new Rectangle3d(pp00, -hd0, vld0);
            Rectangle3d Lmrec00 = new Rectangle3d(mpp00, -hd0, vld0);
            BRec.Add(Rrec00);
            BRec.Add(Rmrec00);
            BRec.Add(Lrec00);
            BRec.Add(Lmrec00);
            EntPts.Add(EP_right00);
            EntPts.Add(EP_left00);
            EntPts.Add(EP_right11);
            EntPts.Add(EP_left11);
          }
          THMidPts.AddRange(PtList, path0);
          EPts.AddRange(EntPts, path0);
        }
      }
      else if (HNumOfBoothsPerSpc == 2)
      {

        HNumOfBoothsPerSpc = 2;
        int nH0 = HNumOfBoothsPerSpc / 2;
        int arrayH0 = Convert.ToInt32(nH0);
        Point3d[] MidPts0 = new Point3d[arrayH0];
        int pathindex0 = 0;
        foreach(List<Rectangle3d> rec in CentralBoothCol.Branches)
        {
          List <Point3d> PtList = new List<Point3d>();
          List <Point3d> HMidPts = new List<Point3d>();
          List <Point3d> EntPts = new List<Point3d>();

          GH_Path path0 = new GH_Path(pathindex0);
          pathindex0 = pathindex0 + 1;
          foreach (Rectangle3d r0 in rec)
          {
            Point3d p00 = r0.PointAt(0.5);
            Point3d p11 = r0.PointAt(2.5);
            Point3d p22 = r0.PointAt(0.0);
            Point3d p33 = r0.PointAt(1.0);

            PtList.Add(p00);
            double vld0 = p00.DistanceTo(p11);
            double hd0 = p22.DistanceTo(p33) * 0.5;
            boothwidth = hd0;
            boothlength = vld0;
            Point3d mp00 = new Point3d(p00.X, p00.Y + vld0, 0.0);
            Point3d EP_right00 = new Point3d(p00.X - hd0 - 0.5, p00.Y + (vld0 * 0.5), 0.0);
            Point3d EP_left00 = new Point3d(p00.X + hd0 + 0.5, p00.Y + (vld0 * 0.5), 0.0);

            PtList.Add(mp00);
            Plane pp00 = new Plane(p00, NormVec);
            Plane mpp00 = new Plane(mp00, NormVec);
            Rectangle3d Rrec00 = new Rectangle3d(pp00, hd0, vld0);

            Rectangle3d Lrec00 = new Rectangle3d(pp00, -hd0, vld0);

            BRec.Add(Rrec00);

            BRec.Add(Lrec00);

            EntPts.Add(EP_right00);
            EntPts.Add(EP_left00);

          }
          THMidPts.AddRange(PtList, path0);
          EPts.AddRange(EntPts, path0);
        }
      }
      else if (HNumOfBoothsPerSpc == 1)

      {
        foreach (List<Rectangle3d> rec in CentralBoothCol.Branches)

        {
          List <Point3d> EntPts = new List<Point3d>();
          foreach (Rectangle3d r0 in rec)
          {
            Point3d P00 = r0.PointAt(3.5);
            Point3d P11 = r0.PointAt(1.5);
            Point3d P22 = r0.PointAt(0.5);
            Point3d P33 = r0.PointAt(2.5);

            Point3d EP00 = new Point3d(P00.X - 0.5, P00.Y, 0.0);
            Point3d EP11 = new Point3d(P11.X + 0.5, P00.Y, 0.0);
            Point3d EP22 = new Point3d(P22.X, P22.Y - 0.5, 0.0);
            Point3d EP33 = new Point3d(P33.X, P33.Y + 0.5, 0.0);
            EntPts.Add(EP00);
            EntPts.Add(EP11);
            EntPts.Add(EP22);
            EntPts.Add(EP33);
            BRec.Add(r0);
          }
          EPts.AddRange(EntPts);
        }

      }

      else
      {
        int nH = HNumOfBoothsPerSpc / 2;
        int arrayH = Convert.ToInt32(nH);
        Point3d[] MidPts = new Point3d[arrayH];
        int pathindex = 0;

        foreach(List<Rectangle3d> rec in CentralBoothCol.Branches)
        {
          List <Point3d> PtList = new List<Point3d>();
          List <Point3d> HMidPts = new List<Point3d>();
          List <Point3d> EntPts = new List<Point3d>();
          GH_Path path = new GH_Path(pathindex);
          pathindex = pathindex + 1;
          foreach (Rectangle3d r0 in rec)
          {
            Point3d p00 = r0.PointAt(0.5);
            Point3d p11 = r0.PointAt(2.5);
            Point3d p22 = r0.PointAt(0.0);
            Point3d p33 = r0.PointAt(1.0);
            PtList.Add(p00);
            PtList.Add(p11);
            Curve crv = Curve.CreateInterpolatedCurve(PtList, 1);
            double[] midpoints = crv.DivideByCount(arrayH, false, out MidPts);
            double VlD = MidPts[0].DistanceTo(MidPts[1]);
            double BoothW = (p22.DistanceTo(p33)) * 0.5;
            foreach(Point3d mp in MidPts)
              for (int j = 0; j < MidPts.Length + 1 ; j++)
              {
                Point3d mp0 = new Point3d(p00.X, p00.Y + (VlD * j), 0.0);
                Point3d Ep_left = new Point3d(p00.X + (0.5 + BoothW), p00.Y + (VlD * j) + (0.5 * VlD), 0.0);
                Point3d Ep_right = new Point3d(p00.X - (0.5 + BoothW), p00.Y + (VlD * j) + (0.5 * VlD), 0.0);
                Plane mpp = new Plane(mp0, NormVec);
                Rectangle3d recLeft = new Rectangle3d(mpp, -BoothW, VlD);
                Rectangle3d recRight = new Rectangle3d(mpp, BoothW, VlD);
                HMidPts.Add(mp0);
                BRec.Add(recLeft);
                BRec.Add(recRight);
                EntPts.Add(Ep_left);
                EntPts.Add(Ep_right);
                boothwidth = BoothW;
                boothlength = VlD;
              }
          }
          THMidPts.AddRange(PtList, path);
          EPts.AddRange(EntPts, path);

        }
      }
    }
    //////////////////////////////////////////////////////////////////////////////////
    //In Case of Hz Aisles only


    //Output
    //    A = HNumOfBoothsPerSpc;
    Booth_Boundary = BRec;
    EntryPts = EPts;
    Booth_W = boothwidth;
    Booth_D = boothlength;

