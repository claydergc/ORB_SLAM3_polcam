import os
import subprocess

executable = "/home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/mono_tum_polcam_auto_aolp"
# executable = "/home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/mono_tum_polcam3"
vocabulary = "/home/ros-noetic/src/ORB_SLAM3_polcam/Vocabulary/ORBvoc.txt"
settings = (
    # "/home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/TRIO50S_1224x1024.yaml"
    "/home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/TRIO50S_606x507.yaml"
    # "/home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/TRIO50S_1224x1024_2.yaml"
)

base_path = "/home/ros-noetic/DATA/datasets/Polcam02/KelvinGrove/"

# sessions = ["20260128/0830/", "20260128/0835/", "20260803/0821/", "20260803/0827/", "20260803/0830/", "20260803/0832/", "20260803/0836/", "20260803/0837/", "20260803/0841/", "20260803/0849/"]
# sessions = ["20260128/0830/", "20260803/0830/", "20260803/0832/"]
sessions = ["20260128/0830/"]
# sessions = ["20260803/0821/", "20260803/0827/", "20260803/0836/", "20260803/0837/"]
# sessions = ["20260803/0821/", "20260803/0827/"]
# sessions = ["20260803/0830/", "20260803/0832/"]
# sessions = ["20260803/0841/", "20260803/0849/"]
# sessions = ["20260803/0841/"]
# sessions = ["20260803/0830/", "20260803/0832/", "20260803/0841/", "20260803/0849/"]
# sessions = ["20260803/0836/", "20260803/0837/"]
#sessions = ["20260128/0830/", "20260128/0835/"]

# resultsBaseFolder = "/home/ros-noetic/DATA/ClayderEvaluation/orbslam3_polcam_tmp_results/"
# resultsBaseFolder = "/home/ros-noetic/DATA/ClayderEvaluation4ICRA2027_iniThFAST10_minThFAST/"
resultsBaseFolder = "/home/ros-noetic/DATA/ClayderEvaluation4ICRA2027_606x507/"

# All combinations of polarized camera images (I0, I45, I90, I135)
# polarization_angles = ["I", "I0", "I45", "I90", "I135"]

# Track failed runs that couldn't be recovered
permanently_failed_runs = []
max_retries = 5  # Maximum number of retries per iteration
iterations_per_session = 10

combo_folder_name = "auto_Itheta0Itheta1_1224x1024"
# combo_folder_name = "auto_Itheta1Itheta0_1224x1024"

total_runs = len(sessions) * iterations_per_session

for session in sessions:
    session_path = os.path.join(base_path, session)

    # Extract location and datetime from session_path
    # Format: .../Location/YYYYMMDD/HHMM/
    path_parts = session_path.rstrip("/").split("/")
    location = path_parts[-3]  # KelvinGrove
    date = path_parts[-2]  # 20260803
    time = path_parts[-1]  # 0836

    # Create parent folder name: first letter of location + date + time
    #location_initial = location[0].lower()  # 'k' from KelvinGrove
    location_initial = "KG"  # 'k' from KelvinGrove
    parent_folder_name = f"{location_initial}_{date}_{time}"
    resultsFolder = os.path.join(resultsBaseFolder, parent_folder_name)

    # Create the parent directory if it doesn't exist
    os.makedirs(resultsFolder, exist_ok=True)

    print(f"Session {session}: results will be saved to: {resultsFolder}")

    trajectoryFileFolder = os.path.join(resultsFolder, combo_folder_name)
    os.makedirs(trajectoryFileFolder, exist_ok=True)
    imgsPolcam = os.path.join(session_path, "polcam") + "/"

    # Run iterations_per_session times for this session
    for iteration in range(iterations_per_session):
        iteration_num = iteration
        retry_count = 0
        success = False

        while not success and retry_count < max_retries:
            if retry_count == 0:
                print(
                    f"  Running iteration {iteration_num}/{iterations_per_session}",
                    end=" ... ",
                )
            else:
                print(f"    Retry {retry_count}/{max_retries - 1}", end=" ... ")

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
                '0.55',
                '60.0',
                '0.22',
                keyFramesTrajectoryFile,  # keyframes trajectory file
                framesTrajectoryFile,  # frames trajectory file
                framesKeyPointsFile,
            ]

            # Run your C++ program and capture both stdout and stderr
            prog_proc = subprocess.run(program, capture_output=True, text=True)

            # Check if process succeeded
            if prog_proc.returncode == 0:
                print("✓ Success")
                success = True
            else:
                # Check stderr for specific error messages
                error_output = prog_proc.stderr + prog_proc.stdout

                if (
                    "X Window System error" in error_output
                    or "BadAccess" in error_output
                    or "MIT-SHM" in error_output
                    or "Gdk-ERROR" in error_output
                ):
                    print(f"❌ X Window System Error")
                    retry_count += 1
                    if retry_count < max_retries:
                        print(
                            f"    Retrying same iteration (attempt {retry_count + 1}/{max_retries})..."
                        )
                    else:
                        print(
                            f"    ⚠️  Max retries ({max_retries}) reached. Moving to next iteration."
                        )
                        permanently_failed_runs.append(
                            {
                                "session": session,
                                "iteration": iteration_num,
                                "error_type": "X Window System Error (unrecoverable)",
                                "trajectory_file": framesTrajectoryFile,
                                "retries_attempted": retry_count,
                            }
                        )
                        success = True  # Exit retry loop but mark as failed
                else:
                    print(f"❌ Process failed (exit code: {prog_proc.returncode})")
                    # Extract first line of error
                    error_msg = (
                        error_output.split("\n")[0] if error_output else "Unknown error"
                    )
                    permanently_failed_runs.append(
                        {
                            "session": session,
                            "iteration": iteration_num,
                            "error_type": f"Exit code {prog_proc.returncode}",
                            "trajectory_file": framesTrajectoryFile,
                            "error_msg": error_msg[:100],
                            "retries_attempted": 0,
                        }
                    )
                    success = (
                        True  # Don't retry for non-X11 errors, move to next iteration
                    )

    print()

print(f"\n{'=' * 70}")
print(f"All {total_runs} iterations completed across {len(sessions)} session(s)!")

if permanently_failed_runs:
    print(f"\n⚠️  {len(permanently_failed_runs)} iteration(s) failed after retries:")
    print(f"{'=' * 70}")
    for failed in permanently_failed_runs:
        print(f"  Session: {failed['session']}, Iteration: {failed['iteration']}")
        print(f"    Error Type: {failed['error_type']}")
        if failed["retries_attempted"] > 0:
            print(f"    Retries attempted: {failed['retries_attempted']}")
        if "error_msg" in failed:
            print(f"    Message: {failed['error_msg']}")
        print()
else:
    print(f"\n✓ All iterations completed successfully!")
