#pragma once

#include <loco_common_mujoco.h>
#include <utils/loco_parsing_element.h>
#include <primitives/loco_single_body_constraint_adapter.h>

namespace loco {
    class TISingleBodyConstraint;
}

namespace loco {
namespace mujoco {

    class TIMujocoSingleBodyConstraintAdapter
    {
    public :

        TIMujocoSingleBodyConstraintAdapter();

        virtual ~TIMujocoSingleBodyConstraintAdapter();

        void SetMjcModel( mjModel* mjc_model_ref ) { m_MjcModelRef = mjc_model_ref; }

        void SetMjcData( mjData* mjc_data_ref ) { m_MjcDataRef = mjc_data_ref; }

        mjModel* mjc_model() { return m_MjcModelRef; }

        const mjModel* mjc_model() const { return m_MjcModelRef; }

        mjData* mjc_data() { return m_MjcDataRef; }

        const mjData* mjc_data() const { return m_MjcDataRef; }

        std::vector<parsing::TElement*> elements_resources();

        std::vector<const parsing::TElement*> elements_resources() const;

        ssize_t mjc_joint_id() const { return m_MjcJointId; }

        ssize_t mjc_joint_qpos_num() const { return m_MjcJointQposNum; }

        ssize_t mjc_joint_qvel_num() const { return m_MjcJointQvelNum; }

        ssize_t mjc_joint_qpos_adr() const { return m_MjcJointQposAdr; }

        ssize_t mjc_joint_qvel_adr() const { return m_MjcJointQvelAdr; }

    protected :

        mjModel* m_MjcModelRef;
        mjData* m_MjcDataRef;

        ssize_t m_MjcJointId;
        ssize_t m_MjcJointQposAdr;
        ssize_t m_MjcJointQvelAdr;
        ssize_t m_MjcJointQposNum;
        ssize_t m_MjcJointQvelNum;

        std::vector<std::unique_ptr<parsing::TElement>> m_MjcfElementsResources;
    };

    class TMujocoSingleBodyRevoluteConstraintAdapter : public TISingleBodyRevoluteConstraintAdapter,
                                                       public TIMujocoSingleBodyConstraintAdapter
    {
    public :

        TMujocoSingleBodyRevoluteConstraintAdapter( TISingleBodyConstraint* constraint_ref )
            : TISingleBodyRevoluteConstraintAdapter( constraint_ref ), TIMujocoSingleBodyConstraintAdapter() {}

        TMujocoSingleBodyRevoluteConstraintAdapter( const TMujocoSingleBodyRevoluteConstraintAdapter& other ) = delete;

        TMujocoSingleBodyRevoluteConstraintAdapter& operator= ( const TMujocoSingleBodyRevoluteConstraintAdapter& other ) = delete;

        ~TMujocoSingleBodyRevoluteConstraintAdapter();

        void Build() override;

        void Initialize() override;

        void Reset() override;

        void OnDetach() override;

        void SetHingeAngle( TScalar hinge_angle ) override;

        void SetLimits( const TVec2& limits ) override;

        void GetHingeAngle( TScalar& dst_hinge_angle ) override;
    };

    class TMujocoSingleBodyPrismaticConstraintAdapter : public TISingleBodyPrismaticConstraintAdapter,
                                                        public TIMujocoSingleBodyConstraintAdapter
    {
    public :

        TMujocoSingleBodyPrismaticConstraintAdapter( TISingleBodyConstraint* constraint_ref )
            : TISingleBodyPrismaticConstraintAdapter( constraint_ref ), TIMujocoSingleBodyConstraintAdapter() {}

        TMujocoSingleBodyPrismaticConstraintAdapter( const TMujocoSingleBodyPrismaticConstraintAdapter& other ) = delete;

        TMujocoSingleBodyPrismaticConstraintAdapter& operator= ( const TMujocoSingleBodyPrismaticConstraintAdapter& other ) = delete;

        ~TMujocoSingleBodyPrismaticConstraintAdapter();

        void Build() override;

        void Initialize() override;

        void Reset() override;

        void OnDetach() override;

        void SetSlidePosition( TScalar slide_position ) override;

        void SetLimits( const TVec2& limits ) override;

        void GetSlidePosition( TScalar& dst_slide_position ) override;
    };

}}