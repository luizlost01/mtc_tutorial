#include <rclcpp/rclcpp.hpp>  // ROS 2 C++ client library
#include <moveit/planning_scene/planning_scene.h>  // Cena de planejamento do MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>  // Interface para modificar a cena
#include <moveit/task_constructor/task.h>  // Classe principal Task do MTC
#include <moveit/task_constructor/solvers.h>  // Planejadores (OMPL, Cartesian, etc)
#include <moveit/task_constructor/stages.h>  // Stages (etapas) do MTC
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

// Logger para mensagens do nó
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
// Alias para facilitar uso do namespace
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);  // Construtor

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();  // Interface do nó para executor

  void doTask();  // Executa a tarefa MTC (planeja e executa)

  void setupPlanningScene();  // Adiciona objetos na cena de planejamento

private:
  // Cria e configura todas as etapas da tarefa MTC
  mtc::Task createTask();
  mtc::Task task_;  // Armazena a tarefa MTC
  rclcpp::Node::SharedPtr node_;  // Nó ROS 2
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }  // Cria nó chamado "mtc_node"
{
}

void MTCTaskNode::setupPlanningScene()
{
  // Cria objeto de colisão para a cena
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";  // ID do objeto a ser manipulado
  object.header.frame_id = "world";  // Frame de referência
  object.primitives.resize(1);  // Um primitivo geométrico
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;  // Forma: cilindro
  object.primitives[0].dimensions = { 0.3, 0.02 };  // Altura: 0.1m, Raio: 0.02m

  // Define posição do objeto no mundo
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;  // 0.5m à frente do robô
  pose.position.y = -0.30;  // 30cm à esquerda
  pose.orientation.w = 1.0;  // Sem rotação (quaternion identidade)
  object.pose = pose;

  // Adiciona objeto na cena de planejamento
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask()
{
  // Cria a tarefa completa com todas as etapas
  task_ = createTask();

  try
  {
    task_.init();  // Inicializa a tarefa (valida etapas, conecta interfaces)
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);  // Erro se inicialização falhar
    return;
  }

  // Planeja até 15 soluções possíveis
  if (!task_.plan(15 /* max_solutions */))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  // Publica solução no RViz para visualização
  task_.introspection().publishSolution(*task_.solutions().front());

  // Executa a primeira solução encontrada
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");  // Nome da tarefa
  task.loadRobotModel(node_);  // Carrega modelo URDF/SRDF do robô

  // Define grupos e frames do robô Panda
  const auto& arm_group_name = "panda_arm";  // Grupo do braço (7 juntas)
  const auto& hand_group_name = "hand";  // Grupo da mão/gripper (dedos)
  const auto& hand_frame = "panda_hand";  // Frame do end-effector

  // Define propriedades globais da tarefa (herdadas por etapas filhas)
  task.setProperty("group", arm_group_name);  // Grupo de planejamento padrão
  task.setProperty("eef", hand_group_name);  // End-effector (efetuador final)
  task.setProperty("ik_frame", hand_frame);  // Frame para cinemática inversa

  // ===== STAGE 1: ESTADO ATUAL =====
  // Captura o estado atual do robô como ponto de partida
  mtc::Stage* current_state_ptr = nullptr;  // Ponteiro para usar em etapas posteriores
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();  // Salva ponteiro antes de mover
  task.add(std::move(stage_state_current));  // Adiciona à tarefa

  // ===== PLANEJADORES =====
  // PipelinePlanner: usa OMPL (sample-based) - bom para movimentos livres no espaço
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  // JointInterpolationPlanner: interpola linearmente no espaço das juntas - rápido para movimentos simples
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  // CartesianPath: movimentos em linha reta no espaço cartesiano
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);  // Velocidade máxima (100%)
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);  // Aceleração máxima (100%)
  cartesian_planner->setStepSize(.01);  // Passo de 1cm para movimentos suaves

  // ===== STAGE 2: ABRIR MÃO =====
  // Move os dedos para a posição aberta antes de pegar o objeto
  // clang-format off
  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  // clang-format on
  stage_open_hand->setGroup(hand_group_name);  // Move apenas o grupo da mão
  stage_open_hand->setGoal("open");  // Usa pose nomeada "open" (definida no SRDF)
  task.add(std::move(stage_open_hand));

  // ===== STAGE 3: MOVER PARA PEGAR =====
  // Connect: conecta estados de etapas anteriores e posteriores, planejando o movimento
  // clang-format off
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  // clang-format on
  stage_move_to_pick->setTimeout(5.0);  // Timeout de 5 segundos para planejamento
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);  // Herda propriedades da tarefa
  task.add(std::move(stage_move_to_pick));

  // ===== STAGE 4: CONTAINER DE PEGAR =====
  // clang-format off
  mtc::Stage* attach_object_stage =
      nullptr;  // Ponteiro para usar na etapa de colocar (place)
  // clang-format on

  // SerialContainer: agrupa múltiplas etapas sequenciais em uma única unidade
  // Isso facilita reutilização e organização do código
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });  // Expõe propriedades
    // clang-format off
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });  // Container herda propriedades
    // clang-format on

    // --- Sub-stage: Aproximar do Objeto ---
    // MoveRelative: move a mão em direção ao objeto
    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      // clang-format on
      stage->properties().set("marker_ns", "approach_object");  // Namespace para marcadores RViz
      stage->properties().set("link", hand_frame);  // Link de referência para o movimento
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });  // Herda grupo
      stage->setMinMaxDistance(0.1, 0.15);  // Move entre 10-15cm

      // Define direção do movimento: Z+ da mão (para frente)
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;  // Em relação ao frame da mão
      vec.vector.z = 1.0;  // Direção Z+ (para frente da mão)
      stage->setDirection(vec);
      grasp->insert(std::move(stage));  // Adiciona ao container
    }

    /****************************************************
     *               GERAR POSE DE PEGAR                *
     ****************************************************/
    // Gera múltiplas poses possíveis ao redor do objeto e calcula IK para cada uma
    {
      // GenerateGraspPose: amostra poses de pegar ao redor do objeto
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");  // Marcadores no RViz
      stage->setPreGraspPose("open");  // Pose da mão antes de pegar
      stage->setObject("object");  // ID do objeto a pegar
      stage->setAngleDelta(M_PI / 30);  // Gera poses a cada 6° ao redor do objeto
      stage->setMonitoredStage(current_state_ptr);  // Monitora estado atual do robô

      // Transformação do frame do objeto para o end-effector
      // Define como a mão deve se alinhar com o objeto
      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();  // Aplica rotações
      grasp_frame_transform.translation().z() = 0.01;  // Offset de 10cm em Z

      // ComputeIK: calcula cinemática inversa para as poses geradas
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(30);  // Até 30 soluções IK por pose
      wrapper->setMinSolutionDistance(1.0);  // Soluções devem diferir por >= 1.0 rad
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);  // Define frame IK
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });  // Herda do pai
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });  // Herda da etapa anterior
      grasp->insert(std::move(wrapper));
    }

    // --- Sub-stage: Permitir Colisões ---
    // Permite que a mão toque o objeto sem gerar erro de colisão
    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);  // true = permitir colisões entre objeto e todos os links da mão
      // clang-format on
      grasp->insert(std::move(stage));
    }

    // --- Sub-stage: Fechar Mão ---
    // Fecha os dedos para segurar o objeto
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);  // Move apenas o grupo da mão
      stage->setGoal("close");  // Usa pose nomeada "close" (definida no SRDF)
      grasp->insert(std::move(stage));
    }

    // --- Sub-stage: Anexar Objeto ---
    // Anexa o objeto ao frame da mão - agora movem juntos
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);  // Objeto agora é parte do robô
      attach_object_stage = stage.get();  // Salva ponteiro para usar na etapa de colocar
      grasp->insert(std::move(stage));
    }

    // --- Sub-stage: Levantar Objeto ---
    // Move a mão (com objeto anexado) para cima
    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      // clang-format on
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });  // Herda grupo
      stage->setMinMaxDistance(0.1, 0.3);  // Levanta entre 10-30cm
      stage->setIKFrame(hand_frame);  // Mantém orientação da mão constante
      stage->properties().set("marker_ns", "lift_object");  // Marcadores no RViz

      // Define direção do movimento: Z+ no mundo (para cima)
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";  // Em relação ao frame do mundo
      vec.vector.z = 1.0;  // Direção Z+ (para cima)
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    task.add(std::move(grasp));  // Adiciona container completo à tarefa
  }
  

  // ===== STAGE 5: MOVER PARA COLOCAR =====
  // Move o robô (com objeto) até a região de colocação
  {
    // clang-format off
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                  { hand_group_name, sampling_planner } });
    // clang-format on
    stage_move_to_place->setTimeout(5.0);  // Timeout de 5 segundos
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);  // Herda propriedades
    task.add(std::move(stage_move_to_place));
  }

  // ===== STAGE 6: CONTAINER DE COLOCAR =====
  // Agrupa etapas para colocar o objeto na posição final
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });  // Expõe propriedades
    // clang-format off
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });  // Herda propriedades
    // clang-format on

    /****************************************************
     *            GERAR POSE DE COLOCAR                 *
     ****************************************************/
    // Define onde e como colocar o objeto
    {
      // GeneratePlacePose: gera pose de destino para colocar o objeto
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");  // Marcadores no RViz
      stage->setObject("object");  // ID do objeto a colocar

      // Define posição de destino em relação ao objeto
      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "object";  // Frame de referência: objeto
      target_pose_msg.pose.position.y = 0.5;  // 50cm na direção Y
      target_pose_msg.pose.orientation.w = 1.0;  // Sem rotação
      stage->setPose(target_pose_msg);  // Define pose alvo
      stage->setMonitoredStage(attach_object_stage);  // Monitora quando objeto foi anexado

      // ComputeIK: calcula cinemática inversa para a pose de colocar
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(2);  // Apenas 2 soluções (place é mais restrito que grasp)
      wrapper->setMinSolutionDistance(1.0);  // Soluções devem diferir por >= 1.0 rad
      wrapper->setIKFrame("object");  // Calcula IK no frame do objeto
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });  // Herda do pai
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });  // Herda da etapa anterior
      place->insert(std::move(wrapper));
    }

    // --- Sub-stage: Abrir Mão ---
    // Abre os dedos para soltar o objeto
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);  // Move apenas o grupo da mão
      stage->setGoal("open");  // Usa pose nomeada "open" - solta o objeto
      place->insert(std::move(stage));
    }

    // --- Sub-stage: Proibir Colisões ---
    // Restaura detecção de colisão entre mão e objeto
    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);  // false = proibir colisões novamente
      // clang-format on
      place->insert(std::move(stage));
    }

    // --- Sub-stage: Desanexar Objeto ---
    // Remove objeto da mão - agora são entidades separadas novamente
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);  // Objeto não é mais parte do robô
      place->insert(std::move(stage));
    }

    // --- Sub-stage: Recuar ---
    // Afasta a mão do objeto após soltá-lo
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });  // Herda grupo
      stage->setMinMaxDistance(0.1, 0.3);  // Recua entre 10-30cm
      stage->setIKFrame(hand_frame);  // Mantém orientação da mão
      stage->properties().set("marker_ns", "retreat");  // Marcadores no RViz

      // Define direção do movimento: X- no mundo (para trás)
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";  // Em relação ao frame do mundo
      vec.vector.x = -0.5;  // Direção X- (para trás)
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    task.add(std::move(place));  // Adiciona container completo à tarefa
  }
  

  // ===== STAGE 7: VOLTAR PARA HOME =====
  // Retorna o braço à posição inicial (home position)
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });  // Herda grupo
    stage->setGoal("ready");  // Usa pose nomeada "ready" (posição inicial do Panda)
    task.add(std::move(stage));
  }
  return task;  // Retorna tarefa completa com todas as 7 etapas
}

// ===== FUNÇÃO MAIN =====
// Ponto de entrada do programa
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);  // Inicializa ROS 2

  // Configura opções do nó
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);  // Declara parâmetros automaticamente

  // Cria nó MTC e executor multi-thread
  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;  // Executor com múltiplas threads

  // Cria thread separada para processar callbacks ROS (spin)
  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());  // Adiciona nó ao executor
    executor.spin();  // Processa callbacks infinitamente
    executor.remove_node(mtc_task_node->getNodeBaseInterface());  // Remove nó ao terminar
  });

  // Executa sequência principal
  mtc_task_node->setupPlanningScene();  // Adiciona objeto cilíndrico na cena
  mtc_task_node->doTask();  // Planeja e executa pick & place completo

  spin_thread->join();  // Aguarda thread de spin terminar
  rclcpp::shutdown();  // Desliga ROS 2
  return 0;
}
