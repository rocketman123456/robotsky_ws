from pathlib import Path

from setuptools import find_packages, setup

package_name = "robotsky_rl_controller"
package_root = Path(__file__).resolve().parent


def collect_model_data_files() -> list[tuple[str, list[str]]]:
    """Collect model artifacts while preserving any subdirectory layout."""
    model_root = package_root / "model"
    if not model_root.exists():
        return []

    grouped_files: dict[str, list[str]] = {}
    for model_path in sorted(model_root.rglob("*")):
        if not model_path.is_file() or model_path.suffix.lower() not in {".pt", ".onnx"}:
            continue

        relative_path = model_path.relative_to(package_root)
        install_dir = Path("share") / package_name / relative_path.parent
        grouped_files.setdefault(str(install_dir), []).append(str(relative_path))

    return sorted(grouped_files.items())


data_files = [
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
]
data_files.extend(collect_model_data_files())

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools", "numpy", "onnxruntime>=1.16.0", "torch"],
    zip_safe=True,
    maintainer="rocketsky",
    maintainer_email="759094438@qq.com",
    description="ROS 2 RL controller node for the RobotSky wheel-legged robot",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rl_controller = robotsky_rl_controller.rl_controller_node:main",
            "wheel_test = robotsky_rl_controller.wheel_test_node:main",
            "joint_test = robotsky_rl_controller.joint_test_node:main",
        ],
    },
)
