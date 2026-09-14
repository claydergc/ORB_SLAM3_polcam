import os
import subprocess

executable = "/home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/mono_tum_polcam_auto_aolp"
# executable = "/home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/mono_tum_polcam3"
vocabulary = "/home/ros-noetic/src/ORB_SLAM3_polcam/Vocabulary/ORBvoc.txt"
settings = (
    "/home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/TRIO50S_1224x1024.yaml"
    # "/home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/TRIO50S_1224x1024_2.yaml"
)

base_path = "/home/ros-noetic/DATA/datasets/Polcam02/KelvinGrove/"

# sessions = ["20260128/0830/", "20260128/0835/", "20260803/0821/", "20260803/0827/", "20260803/0830/", "20260803/0832/", "20260803/0836/", "20260803/0837/", "20260803/0841/", "20260803/0849/"]
sessions = ["20260128/0830/", "20260128/0835/", "20260803/0821/", "20260803/0827/", "20260803/0830/"]
# sessions = ["20260803/0832/", "20260803/0836/", "20260803/0837/", "20260803/0841/", "20260803/0849/"]

# sessions = ["20260128/0830/","20260803/0841/", "20260803/0849/"]
# sessions = ["20260803/0830/","20260803/0821/", "20260803/0827/"]
# sessions = ["20260803/0832/","20260803/0836/","20260803/0837/","20260128/0835/"]

# New sweep parameters (doubles), passed to the executable as argv[4], argv[5], argv[6]
# (i.e. right after imgsPolcam, before the trajectory/keypoints file paths)
#r = [0.5, 1.0]
r = [0.5]
# r = [0.4, 0.45, 0.55, 0.6]
# r = [0.35, 0.65]
# phi_circmax = [70.0, 50.0, 40.0, 30.0]
# phi_circmax = [45.0, 50.0, 55.0, 65.0, 70.0]
# phi_circmax = [45.0, 50.0, 55.0, 65.0]
phi_circmax = [60.0]

d = [0.15, 0.20, 0.25, 0.30]

resultsBaseFolder = "/home/ros-noetic/DATA/AblationStudyICRA2027/orbslam3_polcam_tmp_results/"

# All combinations of polarized camera images (I0, I45, I90, I135)
# polarization_angles = ["I", "I0", "I45", "I90", "I135"]

# Track failed runs
permanently_failed_runs = []
iterations_per_session = 10

total_runs = len(sessions) * len(r) * len(phi_circmax) * len(d) * iterations_per_session

for session in sessions:
    session_path = os.path.join(base_path, session)

    # Extract location and datetime from session_path
    # Format: .../Location/YYYYMMDD/HHMM/
    path_parts = session_path.rstrip("/").split("/")
    location = path_parts[-3]  # KelvinGrove
    date = path_parts[-2]  # 20260803
    time = path_parts[-1]  # 0836

    # Create parent folder name: first letter of location + date + time
    location_initial = location[0].lower()  # 'k' from KelvinGrove
    parent_folder_name = f"{location_initial}_{date}_{time}"
    resultsFolder = os.path.join(resultsBaseFolder, parent_folder_name)

    # Create the parent directory if it doesn't exist
    os.makedirs(resultsFolder, exist_ok=True)

    print(f"Session {session}: results will be saved to: {resultsFolder}")

    imgsPolcam = os.path.join(session_path, "polcam") + "/"

    # Sweep over r, phi_circmax, and d values for this session
    for r_val in r:
        for phi_val in phi_circmax:
            for d_val in d:
                # Give each (r, phi_circmax, d) combo its own trajectory subfolder so
                # results from different sweep values don't overwrite each other
                combo_suffix = f"r{r_val}_phi{phi_val}_d{d_val}"
                trajectoryFileFolder = os.path.join(
                    resultsFolder, combo_suffix
                )

                if os.path.exists(trajectoryFileFolder):
                    print(
                        f"  r={r_val}, phi_circmax={phi_val}, d={d_val} -> folder already exists, "
                        f"skipping (not overwriting): {trajectoryFileFolder}"
                    )
                    print()
                    continue

                os.makedirs(trajectoryFileFolder)

                print(f"  r={r_val}, phi_circmax={phi_val}, d={d_val}")

                # Run iterations_per_session times for this session / r / phi_circmax / d combo
                for iteration in range(iterations_per_session):
                    iteration_num = iteration

                    print(
                        f"    Running iteration {iteration_num}/{iterations_per_session}",
                        end=" ... ",
                    )

                    keyFramesTrajectoryFile = os.path.join(
                        trajectoryFileFolder,
                        str(iteration_num).zfill(5) + "_KeyFramesTrajectory.txt",
                    )

                    framesTrajectoryFile = os.path.join(
                        trajectoryFileFolder,
                        str(iteration_num).zfill(5) + "_FramesTrajectory.txt",
                    )

                    framesKeyPointsFile = os.path.join(
                        trajectoryFileFolder,
                        str(iteration_num).zfill(5) + "_FramesKeypointsNumber.txt",
                    )

                    program = [
                        executable,
                        vocabulary,
                        settings,
                        imgsPolcam,
                        str(r_val),
                        str(phi_val),
                        str(d_val),
                        keyFramesTrajectoryFile,  # keyframes trajectory file
                        framesTrajectoryFile,  # frames trajectory file
                        framesKeyPointsFile,
                    ]

                    # Run your C++ program and capture both stdout and stderr
                    prog_proc = subprocess.run(program, capture_output=True, text=True)

                    # Check if process succeeded
                    if prog_proc.returncode == 0:
                        print("✓ Success")
                    else:
                        print(f"❌ Process failed (exit code: {prog_proc.returncode})")
                        # Extract first line of error
                        error_output = prog_proc.stderr + prog_proc.stdout
                        error_msg = (
                            error_output.split("\n")[0] if error_output else "Unknown error"
                        )
                        permanently_failed_runs.append(
                            {
                                "session": session,
                                "r": r_val,
                                "phi_circmax": phi_val,
                                "d": d_val,
                                "iteration": iteration_num,
                                "error_type": f"Exit code {prog_proc.returncode}",
                                "trajectory_file": framesTrajectoryFile,
                                "error_msg": error_msg[:100],
                            }
                        )

                print()

    print()

print(f"\n{'=' * 70}")
print(
    f"All {total_runs} iterations completed across {len(sessions)} session(s), "
    f"{len(r)} r value(s) x {len(phi_circmax)} phi_circmax value(s) x {len(d)} d value(s)!"
)

if permanently_failed_runs:
    print(f"\n⚠️  {len(permanently_failed_runs)} iteration(s) failed:")
    print(f"{'=' * 70}")
    for failed in permanently_failed_runs:
        print(
            f"  Session: {failed['session']}, r: {failed['r']}, "
            f"phi_circmax: {failed['phi_circmax']}, d: {failed['d']}, Iteration: {failed['iteration']}"
        )
        print(f"    Error Type: {failed['error_type']}")
        if "error_msg" in failed:
            print(f"    Message: {failed['error_msg']}")
        print()
else:
    print(f"\n✓ All iterations completed successfully!")
