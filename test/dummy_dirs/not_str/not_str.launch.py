ComposableNode(
    package="dummy_package",
    plugin="DummyComponent",
    name="dummy_node",
    remappings=[
        ("input", LaunchConfiguration("input")),
        ("output", LaunchConfiguration("output")),
    ],
),
